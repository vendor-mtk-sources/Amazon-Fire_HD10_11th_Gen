/* drivers/misc/lowmemorykiller.c
 *
 * The lowmemorykiller driver lets user-space specify a set of memory thresholds
 * where processes with a range of oom_score_adj values will get killed. Specify
 * the minimum oom_score_adj values in
 * /sys/module/lowmemorykiller/parameters/adj and the number of free pages in
 * /sys/module/lowmemorykiller/parameters/minfree. Both files take a comma
 * separated list of numbers in ascending order.
 *
 * For example, write "0,8" to /sys/module/lowmemorykiller/parameters/adj and
 * "1024,4096" to /sys/module/lowmemorykiller/parameters/minfree to kill
 * processes with a oom_score_adj value of 8 or higher when the free memory
 * drops below 4096 pages and kill processes with a oom_score_adj value of 0 or
 * higher when the free memory drops below 1024 pages.
 *
 * The driver considers memory used for caches to be free, but if a large
 * percentage of the cached memory is locked this can be very inaccurate
 * and processes may not get killed until the normal oom killer is triggered.
 *
 * Copyright (C) 2007-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/oom.h>
#include <linux/sched.h>
#include <linux/swap.h>
#include <linux/rcupdate.h>
#include <linux/profile.h>
#include <linux/notifier.h>
#include <linux/freezer.h>

#define MTK_LMK_USER_EVENT

#ifdef MTK_LMK_USER_EVENT
#include <linux/miscdevice.h>
#endif

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
#include <mt-plat/aee.h>
#include <disp_assert_layer.h>
static u32 in_lowmem;
#endif

#ifdef CONFIG_MTK_ION
#include "mtk/ion_drv.h"
#endif

#ifdef CONFIG_MTK_GPU_SUPPORT
#include <mt-plat/mtk_gpu_utility.h>
#endif

#ifdef CONFIG_64BIT
#define ENABLE_AMR_RAMSIZE	(0x80000)	/* > 2GB */
#else
#define ENABLE_AMR_RAMSIZE	(0x40000)	/* > 1GB */
#endif

static short lowmem_debug_adj;	/* default: 0 */
#ifdef CONFIG_MTK_ENG_BUILD
#ifdef CONFIG_MTK_AEE_FEATURE
static short lowmem_kernel_warn_adj;	/* default: 0 */
#endif
#define output_expect(x) likely(x)
static u32 enable_candidate_log = 1;
#define LMK_LOG_BUF_SIZE 500
static u8 lmk_log_buf[LMK_LOG_BUF_SIZE];
#else
#define output_expect(x) unlikely(x)
static u32 enable_candidate_log;
#endif
static DEFINE_SPINLOCK(lowmem_shrink_lock);

/* fosmod_fireos_crash_reporting begin */
#include<linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#ifdef CONFIG_MTK_GPU_SUPPORT
#include <mt-plat/mtk_gpu_utility.h>
#endif
/* fosmod_fireos_crash_reporting end */

#define CREATE_TRACE_POINTS
#include "trace/lowmemorykiller.h"

#ifndef CONFIG_MTK_DISABLE_LMK_PROMOTE_PRIORITY
#include "internal.h"
#endif

static u32 lowmem_debug_level = 1;
static short lowmem_adj[9] = {
	0,
	1,
	6,
	12,
};

static int lowmem_adj_size = 9;
static int lowmem_minfree[9] = {
	3 * 512,	/* 6MB */
	2 * 1024,	/* 8MB */
	4 * 1024,	/* 16MB */
	16 * 1024,	/* 64MB */
};

static int lowmem_minfree_size = 9;

#define LOWMEM_DEATHPENDING_TIMEOUT	(HZ)
static unsigned long lowmem_deathpending_timeout;

/* fosmod_fireos_crash_reporting begin */
/* Declarations */
size_t ion_mm_heap_total_memory(void);
void ion_mm_heap_memory_detail(void);
void ion_mm_heap_memory_detail_lmk(void);
#ifdef CONFIG_DEBUG_FS
void kbasep_gpu_memory_seq_show_lmk(void);
#endif

/* Constants */
static int BUFFER_SIZE = 16*1024;
static int ELEMENT_SIZE = 256;

/* Variables */
static char *lmk_log_buffer;
static char *buffer_end;
static char *head;
static char *kill_msg_index;
static char *previous_crash;
static int buffer_remaining;
static int foreground_kill;

void lmk_add_to_buffer(const char *fmt, ...)
{
	if (lmk_log_buffer) {
		if (head >= buffer_end) {
			/* Don't add more logs buffer is full */
			return;
		}
		if (buffer_remaining > 0) {
			va_list args;
			int added_size = 0;

			va_start(args, fmt);
			/* If the end of the buffer is reached and the added
			 * value is truncated then vsnprintf will return the
			 * original length of the value instead of the
			 * truncated length - this is intended by design. */
			added_size = vsnprintf(head, buffer_remaining, fmt, args);
			va_end(args);
			if (added_size > 0) {
				/* Add 1 for null terminator */
				added_size = added_size + 1;
				buffer_remaining = buffer_remaining - added_size;
				head = head + added_size;
			}
		}
	}
}

#define lowmem_print(level, x...)			\
	do {						\
		if (lowmem_debug_level >= (level))	\
			pr_warn(x);			\
		if (foreground_kill)			\
			lmk_add_to_buffer(x);			\
	} while (0)

/* In lowmem_print macro only added the lines 'if (foreground_kill)' and 'lmk_add_to_buffer(x)' */
/* fosmod_fireos_crash_reporting end */

static unsigned long lowmem_count(struct shrinker *s,
				  struct shrink_control *sc)
{
#ifdef CONFIG_FREEZER
	/* Do not allow LMK to work when system is freezing */
	if (pm_freezing)
		return 0;
#endif
	return global_page_state(NR_ACTIVE_ANON) +
		global_page_state(NR_ACTIVE_FILE) +
		global_page_state(NR_INACTIVE_ANON) +
		global_page_state(NR_INACTIVE_FILE);
}

#ifdef MTK_LMK_USER_EVENT
static const struct file_operations mtklmk_fops = {
	.owner = THIS_MODULE,
};

static struct miscdevice mtklmk_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mtklmk",
	.fops = &mtklmk_fops,
};

static struct work_struct mtklmk_work;
static int uevent_adj, uevent_minfree;
static void mtklmk_async_uevent(struct work_struct *work)
{
#define MTKLMK_EVENT_LENGTH	(24)
	char adj[MTKLMK_EVENT_LENGTH], free[MTKLMK_EVENT_LENGTH];
	char *envp[3] = { adj, free, NULL };

	snprintf(adj, MTKLMK_EVENT_LENGTH, "OOM_SCORE_ADJ=%d", uevent_adj);
	snprintf(free, MTKLMK_EVENT_LENGTH, "MINFREE=%d", uevent_minfree);
	kobject_uevent_env(&mtklmk_misc.this_device->kobj, KOBJ_CHANGE, envp);
#undef MTKLMK_EVENT_LENGTH
}

static unsigned int mtklmk_initialized;
static unsigned int mtklmk_uevent_timeout = 10000; /* ms */
module_param_named(uevent_timeout, mtklmk_uevent_timeout, uint, 0644);
static void mtklmk_uevent(int oom_score_adj, int minfree)
{
	static unsigned long last_time;
	unsigned long timeout;

	/* change to use jiffies */
	timeout = msecs_to_jiffies(mtklmk_uevent_timeout);

	if (!last_time)
		last_time = jiffies - timeout;

	if (time_before(jiffies, last_time + timeout))
		return;

	last_time = jiffies;

	uevent_adj = oom_score_adj;
	uevent_minfree = minfree;
	schedule_work(&mtklmk_work);
}
#endif

static void dump_memory_status(void)
{
	show_task_mem();
	show_free_areas(0);
#ifdef CONFIG_MTK_ION
	/* Show ION status */
	ion_mm_heap_memory_detail();
#endif
#ifdef CONFIG_MTK_GPU_SUPPORT
	if (mtk_dump_gpu_memory_usage() == false)
		lowmem_print(1, "mtk_dump_gpu_memory_usage not support\n");
#endif
}

static unsigned long lowmem_scan(struct shrinker *s, struct shrink_control *sc)
{
#define LOWMEM_P_STATE_D	(0x1)
#define LOWMEM_P_STATE_R	(0x2)
#define LOWMEM_P_STATE_OTHER	(0x4)

	struct task_struct *tsk;
	struct task_struct *selected = NULL;
	unsigned long rem = 0;
	int tasksize;
	int i;
	short min_score_adj = OOM_SCORE_ADJ_MAX + 1;
	int minfree = 0;
	int selected_tasksize = 0;
	short selected_oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);
	int other_free = global_page_state(NR_FREE_PAGES) - totalreserve_pages;
	int other_file = global_page_state(NR_FILE_PAGES) -
						global_page_state(NR_SHMEM) -
						global_page_state(NR_UNEVICTABLE) -
						total_swapcache_pages();

	int print_extra_info = 0;
	static unsigned long lowmem_print_extra_info_timeout;
	enum zone_type high_zoneidx = gfp_zone(sc->gfp_mask);
	int p_state_is_found = 0;
#ifndef CONFIG_MTK_DISABLE_LMK_PROMOTE_PRIORITY
	int unreclaimable_zones = 0;
#endif
#ifdef CONFIG_SWAP
	int to_be_aggressive = 0;
	unsigned long swap_pages = 0;
	short amr_adj = OOM_SCORE_ADJ_MAX + 1;
#endif
#ifdef CONFIG_MTK_ENG_BUILD
	int pid_dump = -1; /* process to be dump */
	int max_mem = 0;
	static int pid_flm_warn = -1;
	static unsigned long flm_warn_timeout;
	int log_offset = 0, log_ret;
#endif

	/* Subtract CMA free pages from other_free if this is an unmovable page allocation */
	if (IS_ENABLED(CONFIG_CMA))
		if (!(sc->gfp_mask & __GFP_MOVABLE))
			other_free -= global_page_state(NR_FREE_CMA_PAGES);

	if (!spin_trylock(&lowmem_shrink_lock)) {
		lowmem_print(4, "lowmem_shrink lock failed\n");
		return SHRINK_STOP;
	}

	/*
	 * Check whether it is caused by low memory in lower zone(s)!
	 * This will help solve over-reclaiming situation while total number
	 * of free pages is enough, but lower zone(s) is(are) under low memory.
	 */
	if (high_zoneidx < MAX_NR_ZONES - 1) {
		struct pglist_data *pgdat;
		struct zone *z;
		enum zone_type zoneidx;
		unsigned long accumulated_pages = 0, scale = totalram_pages;
		int new_other_free = 0, new_other_file = 0;
		int memory_pressure = 0;

		/* Go through all memory nodes */
		for_each_online_pgdat(pgdat) {
			for (zoneidx = 0; zoneidx <= high_zoneidx; zoneidx++) {
				z = pgdat->node_zones + zoneidx;
				accumulated_pages += z->managed_pages;
				new_other_free += zone_page_state(z, NR_FREE_PAGES);
				new_other_free -= high_wmark_pages(z);
				new_other_file += zone_page_state(z, NR_FILE_PAGES);
				new_other_file -= zone_page_state(z, NR_SHMEM);

				/* Compute memory pressure level */
				memory_pressure += zone_page_state(z, NR_ACTIVE_FILE) +
					zone_page_state(z, NR_INACTIVE_FILE) +
					zone_page_state(z, NR_ACTIVE_ANON) +
					zone_page_state(z, NR_INACTIVE_ANON) +
					new_other_free;

#ifndef CONFIG_MTK_DISABLE_LMK_PROMOTE_PRIORITY
				/* Check whether there is any unreclaimable memory zone */
				if (populated_zone(z) && !zone_reclaimable(z))
					unreclaimable_zones++;
#endif
			}
		}

		/*
		 * Update if we go through ONLY lower zone(s) ACTUALLY
		 * and scale in totalram_pages
		 */
		if (totalram_pages > accumulated_pages) {
			do_div(scale, accumulated_pages);
			if (totalram_pages > accumulated_pages * scale)
				scale += 1;
			new_other_free *= scale;
			new_other_file *= scale;
		}

		/* Update if not kswapd or "being kswapd and high memory pressure" */
		if (!current_is_kswapd() || (current_is_kswapd() && memory_pressure < 0)) {
			other_free = new_other_free;
			other_file = new_other_file;
		}
	}

	/* Let other_free be positive or zero */
	if (other_free < 0) {
		/* lowmem_print(1, "Original other_free [%d] is too low!\n", other_free); */
		other_free = 0;
	}

#if defined(CONFIG_64BIT) && defined(CONFIG_SWAP)
	/* Halve other_free if there is less free swap */
	if (vm_swap_full()) {
		lowmem_print(3, "Halve other_free %d\n", other_free);
		other_free >>= 1;
	}
#endif

#ifdef CONFIG_SWAP
	swap_pages = atomic_long_read(&nr_swap_pages);
	/* More than 1/2 swap usage */
	if (swap_pages * 2 < total_swap_pages)
		to_be_aggressive++;
	/* More than 3/4 swap usage */
	if (swap_pages * 4 < total_swap_pages)
		to_be_aggressive++;

#ifndef CONFIG_MTK_GMO_RAM_OPTIMIZE
	/* Try to enable AMR when we have enough memory */
	if (totalram_pages < ENABLE_AMR_RAMSIZE) {
		to_be_aggressive = 0;
	} else {
		i = lowmem_adj_size - 1 - to_be_aggressive;
		if (to_be_aggressive > 0 && i >= 0)
			amr_adj = lowmem_adj[i];
	}
#endif
#endif

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;
	for (i = 0; i < array_size; i++) {
		minfree = lowmem_minfree[i];
		if (other_free < minfree && other_file < minfree) {
#ifdef CONFIG_SWAP
			if (to_be_aggressive != 0 && i > 3) {
				i -= to_be_aggressive;
				if (i < 3)
					i = 3;
			}
#endif
			min_score_adj = lowmem_adj[i];
			break;
		}
	}

#if defined(CONFIG_SWAP) && !defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
	min_score_adj = min(min_score_adj, amr_adj);
#endif

#ifndef CONFIG_MTK_DISABLE_LMK_PROMOTE_PRIORITY
	/* Promote its priority */
	if (unreclaimable_zones > 0)
		min_score_adj = lowmem_adj[0];
#endif

	lowmem_print(3, "lowmem_scan %lu, %x, ofree %d %d, ma %hd\n",
			sc->nr_to_scan, sc->gfp_mask, other_free,
			other_file, min_score_adj);

	if (min_score_adj == OOM_SCORE_ADJ_MAX + 1) {
		lowmem_print(5, "lowmem_scan %lu, %x, return 0\n",
			     sc->nr_to_scan, sc->gfp_mask);

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
		/* Disable indication if low memory */
		if (in_lowmem) {
			in_lowmem = 0;
			lowmem_print(1, "LowMemoryOff\n");
		}
#endif
		spin_unlock(&lowmem_shrink_lock);
		return 0;
	}

	selected_oom_score_adj = min_score_adj;

	/* More debug log */
	if (output_expect(enable_candidate_log)) {
		if (min_score_adj <= lowmem_debug_adj) {
			if (time_after_eq(jiffies, lowmem_print_extra_info_timeout)) {
				lowmem_print_extra_info_timeout = jiffies + HZ * 2;
				print_extra_info = 1;
			}
		}

		if (print_extra_info) {
			lowmem_print(1, "Free memory other_free: %d, other_file:%d pages\n", other_free, other_file);
#ifdef CONFIG_MTK_ENG_BUILD
			log_offset = snprintf(lmk_log_buf, LMK_LOG_BUF_SIZE, "%s",
					      "<lmk>  pid  score_adj     rss   rswap name\n");
#else
			lowmem_print(1,
				     "<lmk>  pid  score_adj     rss   rswap name\n");
#endif
		}
	}

	rcu_read_lock();
	for_each_process(tsk) {
		struct task_struct *p;
		short oom_score_adj;

		if (tsk->flags & PF_KTHREAD)
			continue;

		if (task_lmk_waiting(tsk) &&
		    time_before_eq(jiffies, lowmem_deathpending_timeout)) {
#ifdef CONFIG_MTK_ENG_BUILD
			static pid_t last_dying_pid;

			if (last_dying_pid != tsk->pid) {
				lowmem_print(1, "lowmem_shrink return directly, due to  %d (%s) is dying\n",
					     tsk->pid, tsk->comm);
				last_dying_pid = tsk->pid;
			}
#endif
			rcu_read_unlock();
			spin_unlock(&lowmem_shrink_lock);
			return SHRINK_STOP;
		} else if (task_lmk_waiting(tsk)) {
#ifdef CONFIG_MTK_ENG_BUILD
			pr_info_ratelimited("%d (%s) is dying, find next candidate\n",
					    tsk->pid, tsk->comm);
#endif
			if (tsk->state == TASK_RUNNING)
				p_state_is_found |= LOWMEM_P_STATE_R;
			else
				p_state_is_found |= LOWMEM_P_STATE_OTHER;

			continue;
		}

		p = find_lock_task_mm(tsk);
		if (!p)
			continue;

#ifdef CONFIG_MTK_AEE_FEATURE
		if (p->signal->flags & SIGNAL_GROUP_COREDUMP) {
			task_unlock(p);
			continue;
		}
#endif
		/* Bypass D-state process */
		if (p->state & TASK_UNINTERRUPTIBLE) {
			lowmem_print(2, "lowmem_scan filter D state process: %d (%s) state:0x%lx\n",
				     p->pid, p->comm, p->state);
			task_unlock(p);
			p_state_is_found |= LOWMEM_P_STATE_D;
			continue;
		}

		oom_score_adj = p->signal->oom_score_adj;

		if (output_expect(enable_candidate_log)) {
			if (print_extra_info) {
#ifdef CONFIG_MTK_ENG_BUILD
log_again:
				log_ret = snprintf(lmk_log_buf + log_offset, LMK_LOG_BUF_SIZE - log_offset,
						   "<lmk>%5d%11d%8lu%8lu %s\n", p->pid,
						   oom_score_adj, get_mm_rss(p->mm),
						   get_mm_counter(p->mm, MM_SWAPENTS), p->comm);

				if ((log_offset + log_ret) >= LMK_LOG_BUF_SIZE || log_ret < 0) {
					*(lmk_log_buf + log_offset) = '\0';
					lowmem_print(1, "\n%s", lmk_log_buf);
					log_offset = 0;
					memset(lmk_log_buf, 0x0, LMK_LOG_BUF_SIZE);
					goto log_again;
				} else {
					log_offset += log_ret;
				}
#else
				lowmem_print(1,	"<lmk>%5d%11d%8lu%8lu %s\n", p->pid,
					     oom_score_adj, get_mm_rss(p->mm),
					     get_mm_counter(p->mm, MM_SWAPENTS), p->comm);
#endif
			}
		}

#ifdef CONFIG_MTK_ENG_BUILD
		tasksize = get_mm_rss(p->mm) + get_mm_counter(p->mm, MM_SWAPENTS);

		/*
		* dump memory info when framework low memory:
		* record the first two pid which consumed most memory.
		*/
		if (tasksize > max_mem) {
			max_mem = tasksize;
			pid_dump = p->pid;
		}

		if (p->pid == pid_flm_warn &&
		    time_before_eq(jiffies, flm_warn_timeout)) {
			task_unlock(p);
			continue;
		}
#endif
		if (oom_score_adj < min_score_adj) {
			task_unlock(p);
			continue;
		}

#ifndef CONFIG_MTK_ENG_BUILD
		tasksize = get_mm_rss(p->mm) + get_mm_counter(p->mm, MM_SWAPENTS);
#endif
		task_unlock(p);
		if (tasksize <= 0)
			continue;
		if (selected) {
			if (oom_score_adj < selected_oom_score_adj)
				continue;
			if (oom_score_adj == selected_oom_score_adj &&
			    tasksize <= selected_tasksize)
				continue;
		}
		selected = p;
		selected_tasksize = tasksize;
		selected_oom_score_adj = oom_score_adj;
		lowmem_print(2, "select '%s' (%d), adj %hd, size %d, to kill\n",
			     p->comm, p->pid, oom_score_adj, tasksize);
	}

#ifdef CONFIG_MTK_ENG_BUILD
	if (log_offset > 0)
		lowmem_print(1, "\n%s", lmk_log_buf);
#endif

/* fosmod_fireos_crash_reporting begin */
#ifndef CONFIG_MTK_ENG_BUILD
	if (lmk_log_buffer && selected && selected_oom_score_adj == 0) {
		foreground_kill = 1;
		head = lmk_log_buffer;
		buffer_remaining = BUFFER_SIZE;
		if (kill_msg_index && previous_crash && buffer_end)
			strncpy(previous_crash, kill_msg_index,
					min((size_t)(buffer_end - kill_msg_index), (size_t)ELEMENT_SIZE));
		lowmem_print(1, "======low memory killer=====\n");
		lowmem_print(1, "Free memory other_free: %d, other_file:%d pages\n", other_free, other_file);
		if (gfp_zone(sc->gfp_mask) == ZONE_NORMAL)
			lowmem_print(1, "ZONE_NORMAL\n");
		else
			lowmem_print(1, "ZONE_HIGHMEM\n");

		rcu_read_lock();
		for_each_process(tsk) {
			struct task_struct *p2;
			short oom_score_adj2;

			if (tsk->flags & PF_KTHREAD)
				continue;

			p2 = find_lock_task_mm(tsk);
			if (!p2)
				continue;

			oom_score_adj2 = p2->signal->oom_score_adj;
#ifdef CONFIG_ZRAM
			lowmem_print(1, "Candidate %d (%s), score_adj %d, rss %lu, rswap %lu, to kill\n",
				p2->pid, p2->comm, oom_score_adj2, get_mm_rss(p2->mm),
				get_mm_counter(p2->mm, MM_SWAPENTS));
#else /* CONFIG_ZRAM */
			lowmem_print(1, "Candidate %d (%s), score_adj %d, rss %lu, to kill\n",
				p2->pid, p2->comm, oom_score_adj2, get_mm_rss(p2->mm));
#endif /* CONFIG_ZRAM */
			task_unlock(p2);
		}
		rcu_read_unlock();
#ifdef CONFIG_MTK_GPU_SUPPORT
#ifdef CONFIG_DEBUG_FS
		kbasep_gpu_memory_seq_show_lmk();
#endif
#endif /* CONFIG_MTK_GPU_SUPPORT */
		kill_msg_index = head;
	}
#endif /* CONFIG_MTK_ENG_BUILD */
/* fosmod_fireos_crash_reporting end */

	if (selected) {
		long cache_size = other_file * (long)(PAGE_SIZE / 1024);
		long cache_limit = minfree * (long)(PAGE_SIZE / 1024);
		long free = other_free * (long)(PAGE_SIZE / 1024);

		task_lock(selected);
		send_sig(SIGKILL, selected, 0);
		if (selected->mm)
			task_set_lmk_waiting(selected);
		task_unlock(selected);
		trace_lowmemory_kill(selected, cache_size, cache_limit, free);
		lowmem_print(1, "Killing '%s' (%d), adj %hd, state(%ld)\n"
				"   to free %ldkB on behalf of '%s' (%d) because\n"
				"   cache %ldkB is below limit %ldkB for oom_score_adj %hd\n"
				"   Free memory is %ldkB above reserved\n"
#ifdef CONFIG_SWAP
				"   (decrease %d level, amr_adj %hd)\n"
#endif
				,
			     selected->comm, selected->pid,
			     selected_oom_score_adj, selected->state,
			     selected_tasksize * (long)(PAGE_SIZE / 1024),
			     current->comm, current->pid,
			     cache_size, cache_limit,
			     min_score_adj,
			     free
#ifdef CONFIG_SWAP
			     , to_be_aggressive, amr_adj
#endif
			     );

		lowmem_deathpending_timeout = jiffies + LOWMEM_DEATHPENDING_TIMEOUT;

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
		/*
		* when kill adj=0 process trigger kernel warning, only in MTK internal eng load
		*/
		if ((selected_oom_score_adj <= lowmem_kernel_warn_adj) && /*lowmem_kernel_warn_adj=16 for test*/
			time_after_eq(jiffies, flm_warn_timeout)) {
			if (pid_dump != pid_flm_warn) {
				#define MSG_SIZE_TO_AEE 70
				char msg_to_aee[MSG_SIZE_TO_AEE];

				lowmem_print(1, "low memory trigger kernel warning\n");
				snprintf(msg_to_aee, MSG_SIZE_TO_AEE,
					 "please contact AP/AF memory module owner[pid:%d]\n", pid_dump);
				aee_kernel_warning_api("LMK", 0, DB_OPT_DEFAULT |
					DB_OPT_DUMPSYS_ACTIVITY |
					DB_OPT_LOW_MEMORY_KILLER |
					DB_OPT_PID_MEMORY_INFO | /* smaps and hprof*/
					DB_OPT_PROCESS_COREDUMP |
					DB_OPT_DUMPSYS_SURFACEFLINGER |
					DB_OPT_DUMPSYS_GFXINFO |
					DB_OPT_DUMPSYS_PROCSTATS,
					"Framework low memory\nCRDISPATCH_KEY:FLM_APAF", msg_to_aee);

				if (pid_dump == selected->pid) {/*select 1st time, filter it*/
					pid_flm_warn = pid_dump;
					flm_warn_timeout = jiffies + 60 * HZ;
					lowmem_print(1, "'%s' (%d) max RSS, not kill\n",
						     selected->comm, selected->pid);
					send_sig(SIGSTOP, selected, 0);
					rcu_read_unlock();
					spin_unlock(&lowmem_shrink_lock);
					dump_memory_status();
					return rem;
				}
			} else {
				lowmem_print(1, "pid_flm_warn:%d, select '%s' (%d)\n",
					     pid_flm_warn, selected->comm, selected->pid);
				pid_flm_warn = -1; /*reset*/
			}
		}
#endif

#if defined(CONFIG_MTK_AEE_FEATURE) && defined(CONFIG_MTK_ENG_BUILD)
		/* Show an indication if low memory */
		if (!in_lowmem && selected_oom_score_adj <= lowmem_debug_adj) {
			in_lowmem = 1;
			lowmem_print(1, "LowMemoryOn\n");
		}
#endif
		rem += selected_tasksize;
/* fosmod_fireos_crash_reporting begin */
		if (foreground_kill) {
				show_free_areas(0);
			#ifdef CONFIG_MTK_ION
				/* Show ION status */
				/* fosmod_fireos_crash_reporting begin */
				lowmem_print(1, "ion_mm_heap_total_memory[%ld]\n", (unsigned long)ion_mm_heap_total_memory());
				ion_mm_heap_memory_detail();
				if (foreground_kill)
					ion_mm_heap_memory_detail_lmk();
				/* fosmod_fireos_crash_reporting end */
			#endif
			#ifdef CONFIG_MTK_GPU_SUPPORT
				if (mtk_dump_gpu_memory_usage() == false)
					lowmem_print(1, "mtk_dump_gpu_memory_usage not support\n");
			#endif
		}
/* fosmod_fireos_crash_reporting end */
	} else {
		if (p_state_is_found & LOWMEM_P_STATE_D)
			lowmem_print(2, "No selected (full of D-state processes at %d)\n", (int)min_score_adj);
		if (p_state_is_found & LOWMEM_P_STATE_R)
			lowmem_print(2, "No selected (full of R-state processes at %d)\n", (int)min_score_adj);
		if (p_state_is_found & LOWMEM_P_STATE_OTHER)
			lowmem_print(2, "No selected (full of OTHER-state processes at %d)\n", (int)min_score_adj);
	}

	lowmem_print(4, "lowmem_scan %lu, %x, return %lu\n",
		     sc->nr_to_scan, sc->gfp_mask, rem);
	rcu_read_unlock();
	spin_unlock(&lowmem_shrink_lock);

	/* Dump HW memory outside the lock */
	if (selected && output_expect(enable_candidate_log))
		if (print_extra_info)
			dump_memory_status();

#ifdef MTK_LMK_USER_EVENT
	/* Send uevent if needed */
	if (mtklmk_initialized && current_is_kswapd() && mtklmk_uevent_timeout)
		mtklmk_uevent(min_score_adj, minfree);
#endif

	foreground_kill = 0; /* fosmod_fireos_crash_reporting oneline */

	return rem;

#undef LOWMEM_P_STATE_D
#undef LOWMEM_P_STATE_R
#undef LOWMEM_P_STATE_OTHER
}

/* fosmod_fireos_crash_reporting begin */
static int lowmem_proc_show(struct seq_file *m, void *v)
{
	char *ptr;

	if (!lmk_log_buffer) {
		seq_printf(m, "lmk_logs are not functioning - something went wrong during init");
		return 0;
	}
	ptr = lmk_log_buffer;
	while (ptr < head) {
		int cur_line_len = strlen(ptr);
		seq_printf(m, ptr, "\n");
		if (cur_line_len <= 0)
			break;
		/* add 1 to skip the null terminator for C Strings */
		ptr = ptr + cur_line_len + 1;
	}
	if (previous_crash && previous_crash[0] != '\0') {
		seq_printf(m, "previous crash:\n");
		seq_printf(m, previous_crash, "\n");
	}
	return 0;
}

static int lowmem_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, lowmem_proc_show, NULL);
}

static const struct file_operations lowmem_proc_fops = {
	.open       = lowmem_proc_open,
	.read       = seq_read,
	.release    = single_release
};
/* fosmod_fireos_crash_reporting end */
static struct shrinker lowmem_shrinker = {
	.scan_objects = lowmem_scan,
	.count_objects = lowmem_count,
	.seeks = DEFAULT_SEEKS * 16
};

static int __init lowmem_init(void)
{
#ifndef CONFIG_MTK_DISABLE_LMK_PROMOTE_PRIORITY
#if defined(CONFIG_ZRAM) && defined(CONFIG_MTK_GMO_RAM_OPTIMIZE)
	vm_swappiness = 100;
#endif
#else
#ifdef CONFIG_ZRAM
#ifdef CONFIG_MTK_GMO_RAM_OPTIMIZE
	vm_swappiness = 150;
#else
	vm_swappiness = 100;
#endif
#endif
#endif
	register_shrinker(&lowmem_shrinker);

    /* fosmod_fireos_crash_reporting begin */
	proc_create("lmk_logs", 0, NULL, &lowmem_proc_fops);
	lmk_log_buffer = kzalloc(BUFFER_SIZE, GFP_KERNEL);
	if (lmk_log_buffer) {
		buffer_end = lmk_log_buffer + BUFFER_SIZE;
		head = lmk_log_buffer;
		buffer_remaining = BUFFER_SIZE;
		foreground_kill = 0;
		kill_msg_index = NULL;
		previous_crash = kzalloc(ELEMENT_SIZE, GFP_KERNEL);
		if (!previous_crash)
			printk(KERN_ALERT "unable to allocate previous_crash for /proc/lmk_logs - previous_crash will not be logged");
	} else {
		printk(KERN_ALERT "unable to allocate buffer for /proc/lmk_logs - feature will be disabled");
	}
    /* fosmod_fireos_crash_reporting end */

#ifdef MTK_LMK_USER_EVENT
	/* initialize work for uevent */
	INIT_WORK(&mtklmk_work, mtklmk_async_uevent);

	/* register as misc device */
	if (!misc_register(&mtklmk_misc)) {
		pr_info("%s: successful to register misc device!\n", __func__);
		mtklmk_initialized = 1;
	}
#endif

	return 0;
}
device_initcall(lowmem_init);

#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
static short lowmem_oom_adj_to_oom_score_adj(short oom_adj)
{
	if (oom_adj == OOM_ADJUST_MAX)
		return OOM_SCORE_ADJ_MAX;
	else
		return (oom_adj * OOM_SCORE_ADJ_MAX) / -OOM_DISABLE;
}

static void lowmem_autodetect_oom_adj_values(void)
{
	int i;
	short oom_adj;
	short oom_score_adj;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;

	if (array_size <= 0)
		return;

	oom_adj = lowmem_adj[array_size - 1];
	if (oom_adj > OOM_ADJUST_MAX)
		return;

	oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
	if (oom_score_adj <= OOM_ADJUST_MAX)
		return;

	lowmem_print(1, "lowmem_shrink: convert oom_adj to oom_score_adj:\n");
	for (i = 0; i < array_size; i++) {
		oom_adj = lowmem_adj[i];
		oom_score_adj = lowmem_oom_adj_to_oom_score_adj(oom_adj);
		lowmem_adj[i] = oom_score_adj;
		lowmem_print(1, "oom_adj %d => oom_score_adj %d\n",
			     oom_adj, oom_score_adj);
	}
}

static int lowmem_adj_array_set(const char *val, const struct kernel_param *kp)
{
	int ret;

	ret = param_array_ops.set(val, kp);

	/* HACK: Autodetect oom_adj values in lowmem_adj array */
	lowmem_autodetect_oom_adj_values();

	return ret;
}

static int lowmem_adj_array_get(char *buffer, const struct kernel_param *kp)
{
	return param_array_ops.get(buffer, kp);
}

static void lowmem_adj_array_free(void *arg)
{
	param_array_ops.free(arg);
}

static struct kernel_param_ops lowmem_adj_array_ops = {
	.set = lowmem_adj_array_set,
	.get = lowmem_adj_array_get,
	.free = lowmem_adj_array_free,
};

static const struct kparam_array __param_arr_adj = {
	.max = ARRAY_SIZE(lowmem_adj),
	.num = &lowmem_adj_size,
	.ops = &param_ops_short,
	.elemsize = sizeof(lowmem_adj[0]),
	.elem = lowmem_adj,
};
#endif

/*
 * get_min_free_pages
 * returns the low memory killer watermark of the given pid,
 * When the system free memory is lower than the watermark, the LMK (low memory
 * killer) may try to kill processes.
 */
int get_min_free_pages(pid_t pid)
{
	struct task_struct *p;
	int target_oom_adj = 0;
	int i = 0;
	int array_size = ARRAY_SIZE(lowmem_adj);

	if (lowmem_adj_size < array_size)
		array_size = lowmem_adj_size;
	if (lowmem_minfree_size < array_size)
		array_size = lowmem_minfree_size;

	for_each_process(p) {
		/* search pid */
		if (p->pid == pid) {
			task_lock(p);
			target_oom_adj = p->signal->oom_score_adj;
			task_unlock(p);
			/* get min_free value of the pid */
			for (i = array_size - 1; i >= 0; i--) {
				if (target_oom_adj >= lowmem_adj[i]) {
					pr_debug("pid: %d, target_oom_adj = %d, lowmem_adj[%d] = %d, lowmem_minfree[%d] = %d\n",
						 pid, target_oom_adj, i, lowmem_adj[i], i, lowmem_minfree[i]);
					return lowmem_minfree[i];
				}
			}
			goto out;
		}
	}

out:
	lowmem_print(3, "[%s]pid: %d, adj: %d, lowmem_minfree = 0\n",
		     __func__, pid, p->signal->oom_score_adj);
	return 0;
}
EXPORT_SYMBOL(get_min_free_pages);

/* Query LMK minfree settings */
/* To query default value, you can input index with value -1. */
size_t query_lmk_minfree(int index)
{
	int which;

	/* Invalid input index, return default value */
	if (index < 0)
		return lowmem_minfree[2];

	/* Find a corresponding output */
	which = 5;
	do {
		if (lowmem_adj[which] <= index)
			break;
	} while (--which >= 0);

	/* Fix underflow bug */
	which = (which < 0) ? 0 : which;

	return lowmem_minfree[which];
}
EXPORT_SYMBOL(query_lmk_minfree);

/*
 * not really modular, but the easiest way to keep compat with existing
 * bootargs behaviour is to continue using module_param here.
 */
module_param_named(cost, lowmem_shrinker.seeks, int, S_IRUGO | S_IWUSR);
#ifdef CONFIG_ANDROID_LOW_MEMORY_KILLER_AUTODETECT_OOM_ADJ_VALUES
module_param_cb(adj, &lowmem_adj_array_ops,
		.arr = &__param_arr_adj,
		S_IRUGO | S_IWUSR);
__MODULE_PARM_TYPE(adj, "array of short");
#else
module_param_array_named(adj, lowmem_adj, short, &lowmem_adj_size,
			 S_IRUGO | S_IWUSR);
#endif
module_param_array_named(minfree, lowmem_minfree, uint, &lowmem_minfree_size,
			 S_IRUGO | S_IWUSR);
module_param_named(debug_level, lowmem_debug_level, uint, S_IRUGO | S_IWUSR);
module_param_named(debug_adj, lowmem_debug_adj, short, S_IRUGO | S_IWUSR);
module_param_named(candidate_log, enable_candidate_log, uint, S_IRUGO | S_IWUSR);
