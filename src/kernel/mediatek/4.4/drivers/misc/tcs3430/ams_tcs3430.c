/*
*****************************************************************************
* Copyright by ams AG                                                       *
* All rights are reserved.                                                  *
*                                                                           *
* IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
* THE SOFTWARE.                                                             *
*                                                                           *
* THIS SOFTWARE IS PROVIDED FOR USE ONLY IN CONJUNCTION WITH AMS PRODUCTS.  *
* USE OF THE SOFTWARE IN CONJUNCTION WITH NON-AMS-PRODUCTS IS EXPLICITLY    *
* EXCLUDED.                                                                 *
*                                                                           *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
*****************************************************************************
*/

/*! \file
* \brief Device driver for monitoring ambient light intensity in (lux)
* functionality within the AMS-TAOS TCS3430 family of devices.
*/

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/unistd.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/gpio/consumer.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif

#include <linux/i2c/ams_tcs3430.h>
#include "ams_i2c.h"
#include "ams_tcs3430_als.h"

/* TCS3430 Identifiers */
static u8 const tcs3430_ids[] = {
//    ID,    REVID,    REVID2
	0xDC,    0x01, 0x00,
};

/* TCS3430 Device Names */
static char const *tcs3430_names[] = {
	"tcs3430",
};

/* Registers to restore */
static u8 const restorable_regs[] = {
	TCS3430_REG_PERS,
	TCS3430_REG_CFG0,
	TCS3430_REG_CFG1,
	TCS3430_REG_ATIME,
};

static int tcs3430_irq_handler(struct tcs3430_chip *chip)
{
	u8 status;
	int ret;

	AMS_MUTEX_LOCK(&chip->lock);
	ret = ams_i2c_read(chip->client,
			TCS3430_REG_STATUS,
			&chip->shadow[TCS3430_REG_STATUS]);
	status = chip->shadow[TCS3430_REG_STATUS];

	if (status == 0) {
		AMS_MUTEX_UNLOCK(&chip->lock);
		return 0;  // not our interrupt
	}

	do {

		// Clear the interrupts we'll process
		ams_i2c_write_direct(chip->client, TCS3430_REG_STATUS, status);

		if (status & TCS3430_ST_ALS_SAT)
			chip->in_asat = true;
		else
			chip->in_asat = false;

		/*************/
		/* ALS       */
		/*************/
		if ((status & TCS3430_ST_ALS_IRQ) || chip->in_asat)
		{
			tcs3430_read_als(chip);
			tcs3430_report_als(chip);
		}

		ret = ams_i2c_read(chip->client,
				TCS3430_REG_STATUS,
				&chip->shadow[TCS3430_REG_STATUS]);
		status = chip->shadow[TCS3430_REG_STATUS];
	} while (status != 0);
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 1;  // we handled the interrupt
}

static irqreturn_t tcs3430_irq(int irq, void *handle)
{
	struct tcs3430_chip *chip = handle;
	struct device *dev = &chip->client->dev;
	int ret;

	if (chip->in_suspend) {
		dev_info(dev, "%s: in suspend\n", __func__);
		chip->irq_pending = 1;
		ret = 0;
		goto bypass;
	}
	ret = tcs3430_irq_handler(chip);

bypass:
	return ret ? IRQ_HANDLED : IRQ_NONE;
}

static int tcs3430_flush_regs(struct tcs3430_chip *chip)
{
	unsigned i;
	int rc;
	u8 reg;

	for (i = 0; i < ARRAY_SIZE(restorable_regs); i++) {
		reg = restorable_regs[i];
		rc = ams_i2c_write(chip->client, chip->shadow, reg, chip->shadow[reg]);
		if (rc) {
			dev_err(&chip->client->dev, "%s: err on reg 0x%02x\n",
					__func__, reg);
			break;
		}
	}

	return rc;
}

static int tcs3430_pltf_power_on(struct tcs3430_chip *chip)
{
	int rc = 0;
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
				POWER_ON);
		usleep_range(10000, 11000);
	}
	chip->unpowered = rc != 0;
	dev_info(&chip->client->dev, "\n\n%s: unpowered=%d\n",
			__func__, chip->unpowered);
	return rc;
}

static int tcs3430_pltf_power_off(struct tcs3430_chip *chip)
{
	int rc = 0;
	if (chip->pdata->platform_power) {
		rc = chip->pdata->platform_power(&chip->client->dev,
			POWER_OFF);
		chip->unpowered = rc == 0;
	} else {
		chip->unpowered = false;
	}
	dev_info(&chip->client->dev, "%s: unpowered=%d\n",
		__func__, chip->unpowered);
	return rc;
}

static void tcs3430_set_defaults(struct tcs3430_chip *chip)
{
	u8 *sh = chip->shadow;
	struct device *dev = &chip->client->dev;

	// Clear the register shadow area
	memset(chip->shadow, 0x00, sizeof(chip->shadow));

	// If there is platform data use it
	if (chip->pdata) {
		dev_info(dev, "%s: Loading pltform data\n", __func__);
		chip->params.persist = chip->pdata->parameters.persist;
		chip->params.als_gain = chip->pdata->parameters.als_gain;
		chip->params.als_auto_gain = chip->pdata->parameters.als_auto_gain;
		chip->params.als_deltaP = chip->pdata->parameters.als_deltaP;
		chip->params.als_time = chip->pdata->parameters.als_time;
	} else {
		dev_info(dev, "%s: use defaults\n", __func__);
		chip->params.persist = ALS_PERSIST(2);
		chip->params.als_gain = AGAIN_4;
		chip->params.als_deltaP = 10;
		chip->params.als_time = AW_TIME_MS(200);
		chip->params.als_auto_gain = false;
	}

	// Copy the default values into the register shadow area
	sh[TCS3430_REG_PERS]     = chip->params.persist;
	sh[TCS3430_REG_ATIME]    = chip->params.als_time;
	sh[TCS3430_REG_CFG1]     = chip->params.als_gain << TCS3430_SHIFT_AGAIN;
	sh[TCS3430_REG_CFG2]     = 0x04;

	tcs3430_flush_regs(chip);
}

static int tcs3430_add_sysfs_interfaces(struct device *dev,
                                        struct device_attribute *a,
                                        int size)
{
	int i;
	for (i = 0; i < size; i++)
		if (device_create_file(dev, a + i))
			goto undo;
	return 0;
undo:
	for (; i >= 0 ; i--)
		device_remove_file(dev, a + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void tcs3430_remove_sysfs_interfaces(struct device *dev,
    struct device_attribute *a, int size)
{
	int i;
	for (i = 0; i < size; i++)
		device_remove_file(dev, a + i);
}

static int tcs3430_get_id(struct tcs3430_chip *chip, u8 *id, u8 *rev, u8 *rev2)
{

	*rev2 = 0; //TCS3430 does not have a REVID2 register
	ams_i2c_read(chip->client, TCS3430_REG_REVID, rev);
	ams_i2c_read(chip->client, TCS3430_REG_ID, id);

	return 0;
}

static int tcs3430_power_on(struct tcs3430_chip *chip)
{
	int rc;
	rc = tcs3430_pltf_power_on(chip);
	if (rc)
		return rc;
	dev_info(&chip->client->dev, "%s: chip was off, restoring regs\n",
			__func__);
	return tcs3430_flush_regs(chip);
}

/************************************************/
/* Specific Device Setup.  Configured in Probe. */
/************************************************/
static int tcs3430_als_idev_open(struct input_dev *idev)
{
	struct tcs3430_chip *chip = dev_get_drvdata(&idev->dev);
	int rc = 0;

	dev_info(&idev->dev, "%s\n", __func__);
	AMS_MUTEX_LOCK(&chip->lock);
	if (chip->unpowered) {
		rc = tcs3430_power_on(chip);
		if (rc)
			goto chip_on_err;
	}
	rc = tcs3430_configure_als_mode(chip, 1);
	if (rc)
		tcs3430_pltf_power_off(chip);
chip_on_err:
	AMS_MUTEX_UNLOCK(&chip->lock);
	return 0;//rc;
}

static void tcs3430_als_idev_close(struct input_dev *idev)
{
	struct tcs3430_chip *chip = dev_get_drvdata(&idev->dev);
	dev_info(&idev->dev, "%s\n", __func__);

	AMS_MUTEX_LOCK(&chip->lock);
	tcs3430_configure_als_mode(chip, 0);
	tcs3430_pltf_power_off(chip);
	AMS_MUTEX_UNLOCK(&chip->lock);
}

#ifdef CONFIG_OF
int tcs3430_init_dt(struct tcs3430_i2c_platform_data *pdata)
{
	struct device_node *np = pdata->of_node;
	const char *str;
	u32 val;

	if (!pdata->of_node)
		return 0;

	if (!of_property_read_string(np, "als_name", &str))
		pdata->als_name = str;

	if (!of_property_read_u32(np, "persist", &val))
		pdata->parameters.persist = val;

	if (!of_property_read_u32(np, "als_gain", &val))
		pdata->parameters.als_gain = val;

	if (!of_property_read_u32(np, "als_auto_gain", &val))
		pdata->parameters.als_auto_gain = val;

	if (!of_property_read_u32(np, "als_deltap", &val))
		pdata->parameters.als_deltaP = val;

	if (!of_property_read_u32(np, "als_time", &val))
		pdata->parameters.als_time = val;

	return 0;
}

static const struct of_device_id tcs3430_of_match[] = {
	{ .compatible = "ams,tcs3430" },
	{ }
};
MODULE_DEVICE_TABLE(of, tcs3430_of_match);
#endif

static int tcs3430_probe(struct i2c_client *client,
                         const struct i2c_device_id *idp)
{
	int i, ret;
	u8 id, rev, rev2;
	struct device *dev = &client->dev;
	static struct tcs3430_chip *chip;
	struct tcs3430_i2c_platform_data *pdata = dev->platform_data;
	bool powered = 0;
	unsigned long default_irq_trigger = 0;

	/****************************************/
	/* Validate bus and device registration */
	/****************************************/

	dev_info(dev, "%s: client->irq = %d\n", __func__, client->irq);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(dev, "%s: i2c smbus byte data unsupported\n", __func__);
		ret = -EOPNOTSUPP;
		goto init_failed;
	}

#ifdef CONFIG_OF
	if (!pdata) {
		pdata = devm_kzalloc(dev, sizeof(struct tcs3430_i2c_platform_data),
				GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;
		if (of_match_device(tcs3430_of_match, &client->dev)) {
			pdata->of_node = client->dev.of_node;
			ret = tcs3430_init_dt(pdata);
			if (ret)
				return ret;
		}
	}
#endif

	if (pdata->platform_init) {
		ret = pdata->platform_init();
		if (ret)
			goto init_failed;
	}
	if (pdata->platform_power) {
		ret = pdata->platform_power(dev, POWER_ON);
		if (ret) {
			dev_err(dev, "%s: pltf power on failed\n", __func__);
			goto pon_failed;
		}
		powered = true;
		mdelay(10);
	}
	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		ret = -ENOMEM;
		goto malloc_failed;
	}

	mutex_init(&chip->lock);
	chip->client = client;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);

	/********************************************************************/
	/* Validate the appropriate ams device is available for this driver */
	/********************************************************************/

	ret = tcs3430_get_id(chip, &id, &rev, &rev2);

	dev_info(dev, "%s: device id:%02x device revid:%02x device revid_2:%02x\n",
			__func__, id, rev, rev2);

	id &= 0xfc;
	rev &= 0x07;
	for (i = 0; i < ARRAY_SIZE(tcs3430_ids)/3; i++) {
		if (id == (tcs3430_ids[i*3+0]))
			if (rev == (tcs3430_ids[i*3+1]))
				if (rev2 == (tcs3430_ids[i*3+2]))
					break;
	}
	if (i < ARRAY_SIZE(tcs3430_names)) {
		dev_info(dev, "%s: '%s revid_2: 0x%x' detected\n", __func__,
				tcs3430_names[i], rev2);
		chip->device_index = i;
	} else {
		dev_err(dev, "%s: not supported chip id\n", __func__);
		ret = -EOPNOTSUPP;
		goto id_failed;
	}

	/*********************/
	/* Set chip defaults */
	/*********************/

	tcs3430_set_defaults(chip);
	ret = tcs3430_flush_regs(chip);
	if (ret)
		goto flush_regs_failed;
	if (pdata->platform_power) {
		pdata->platform_power(dev, POWER_OFF);
		powered = false;
		chip->unpowered = true;
	}

	/*********************/
	/* Initialize ALS    */
	/*********************/

	if (!pdata->als_name)
		goto bypass_als_idev;
	chip->a_idev = devm_input_allocate_device(dev);
	if (!chip->a_idev) {
		dev_err(dev, "%s: no memory for input_dev '%s'\n",
				__func__, pdata->als_name);
		ret = -ENODEV;
		goto input_a_alloc_failed;
	}
	chip->a_idev->name = pdata->als_name;
	chip->a_idev->id.bustype = BUS_I2C;
	set_bit(EV_ABS, chip->a_idev->evbit);
	set_bit(ABS_MISC, chip->a_idev->absbit);
	input_set_abs_params(chip->a_idev, ABS_MISC, 0, 65535, 0, 0);
	chip->a_idev->open = tcs3430_als_idev_open;
	chip->a_idev->close = tcs3430_als_idev_close;
	input_set_drvdata(chip->a_idev, chip);
	ret = input_register_device(chip->a_idev);
	if (ret) {
		dev_err(dev, "%s: cant register input '%s'\n",
				__func__, pdata->als_name);
		goto input_a_alloc_failed;
	}
	ret = tcs3430_add_sysfs_interfaces(&chip->a_idev->dev,
			tcs3430_als_attrs, tcs3430_als_attrs_size);
	if (ret)
		goto input_a_sysfs_failed;

bypass_als_idev:

	/****************************/
	/* Initialize IRQ & Handler */
	/****************************/

	default_irq_trigger =
		irqd_get_trigger_type(irq_get_irq_data(client->irq));
	ret = devm_request_threaded_irq(dev, client->irq,
			NULL, &tcs3430_irq,
			default_irq_trigger |
			IRQF_SHARED         |
			IRQF_ONESHOT,
			dev_name(dev), chip);
	if (ret) {
		dev_err(dev, "Failed to request irq %d\n", client->irq);
		goto irq_register_fail;
	}

	// Power up device
	ams_i2c_write(chip->client, chip->shadow, TCS3430_REG_ENABLE, 0x01);

	dev_info(dev, "Probe ok.\n");
	return 0;

	/************************************************************************/
	/* Exit points for device functional failures (Prox, ALS)               */
	/* This must be unwound in the correct order,                           */
	/* reverse from initialization above                                    */
	/************************************************************************/

irq_register_fail:
	if (chip->a_idev)
	{
		tcs3430_remove_sysfs_interfaces(&chip->a_idev->dev,
				tcs3430_als_attrs, tcs3430_als_attrs_size);
input_a_sysfs_failed:
		input_unregister_device(chip->a_idev);
	}

	/******************************************************************/
	/* Exit points for general device initialization failures         */
	/******************************************************************/

input_a_alloc_failed:
flush_regs_failed:
id_failed:
	i2c_set_clientdata(client, NULL);
malloc_failed:
	if (powered && pdata->platform_power)
		pdata->platform_power(dev, POWER_OFF);
pon_failed:
	if (pdata->platform_teardown)
		pdata->platform_teardown(dev);
init_failed:
	dev_err(dev, "Probe failed.\n");
	return ret;
}

static int tcs3430_suspend(struct device *dev)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	/* struct tcs3430_i2c_platform_data *pdata = dev->platform_data; */

	printk(KERN_ERR "\nTCS3430: suspend()\n");
	dev_info(dev, "%s\n", __func__);
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 1;

	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 1);
	} else if (!chip->unpowered) {
		dev_info(dev, "powering off\n");
		tcs3430_pltf_power_off(chip);
	}
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

static int tcs3430_resume(struct device *dev)
{
	struct tcs3430_chip *chip = dev_get_drvdata(dev);
	bool als_on;

	return 0;
	printk(KERN_ERR "\nTCS3430: resume()\n");
	AMS_MUTEX_LOCK(&chip->lock);
	chip->in_suspend = 0;

	dev_info(dev, "%s: powerd %d, als: needed %d  enabled %d",
			__func__, !chip->unpowered, als_on,
			chip->als_enabled);
	if (chip->wake_irq) {
		irq_set_irq_wake(chip->client->irq, 0);
		chip->wake_irq = 0;
	}

	/* err_power: */
	AMS_MUTEX_UNLOCK(&chip->lock);

	return 0;
}

static int tcs3430_remove(struct i2c_client *client)
{
	struct tcs3430_chip *chip = i2c_get_clientdata(client);

	dev_info(&client->dev, "%s\n", __func__);
	if (chip->gpiod_interrupt) {
		devm_free_irq(&client->dev, client->irq, chip);
	}
	if (chip->a_idev) {
		tcs3430_remove_sysfs_interfaces(&chip->a_idev->dev,
				tcs3430_als_attrs, tcs3430_als_attrs_size);
		input_unregister_device(chip->a_idev);
	}
	if (chip->pdata->platform_teardown)
		chip->pdata->platform_teardown(&client->dev);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static struct i2c_device_id tcs3430_idtable[] = {
	{ "tcs3430", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, tcs3430_idtable);

static const struct dev_pm_ops tcs3430_pm_ops = {
	.suspend = tcs3430_suspend,
	.resume  = tcs3430_resume,
};

static struct i2c_driver tcs3430_driver = {
	.driver = {
		.name = "tcs3430",
		.pm = &tcs3430_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(tcs3430_of_match),
#endif
	},
	.id_table = tcs3430_idtable,
	.probe = tcs3430_probe,
	.remove = tcs3430_remove,
};

module_i2c_driver(tcs3430_driver);

MODULE_AUTHOR("AMS AOS Software<cs.americas@ams.com>");
MODULE_DESCRIPTION("AMS-TAOS tcs3430 ALS, Color sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.2");
