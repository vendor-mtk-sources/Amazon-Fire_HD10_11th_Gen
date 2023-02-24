/************************************************************************
* Copyright (C) 2012-2019, Focaltech Systems (R), All Rights Reserved.
*
* File Name: Focaltech_test_ft5822.c
*
* Author: Focaltech Driver Team
*
* Created: 2015-07-14
*
* Abstract:
*
************************************************************************/

/*****************************************************************************
* Included header files
*****************************************************************************/
#include "../focaltech_test.h"

/*****************************************************************************
* Private constant and macro definitions using #define
*****************************************************************************/

/*****************************************************************************
* Private enumerations, structures and unions using typedef
*****************************************************************************/

/*****************************************************************************
* Static variables
*****************************************************************************/

/*****************************************************************************
* Global variable or extern global variabls/functions
*****************************************************************************/

/*****************************************************************************
* Static function prototypes
*****************************************************************************/
static int ft5822_rawdata_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    bool result = false;
    bool result2 = false;
    u8 fre = 0;
    u8 fir = 0;
    u8 normalize = 0;
    int *rawdata = NULL;
    int *rawdata_tmp = NULL;
    int rawdata_cnt = 0;
    struct mc_sc_threshold *thr = &tdata->ic.mc_sc.thr;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("\n============ Test Item: rawdata test\n");
    memset(tdata->buffer, 0, tdata->buffer_length);
    rawdata = tdata->buffer;

    if (!thr->rawdata_h_min || !thr->rawdata_h_max || !test_result) {
        FTS_TEST_SAVE_ERR("rawdata_h_min/max test_result is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    if (!thr->rawdata_l_min || !thr->rawdata_l_max) {
        FTS_TEST_SAVE_ERR("rawdata_l_min/max is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        goto test_err;
    }

    /* rawdata test in mapping mode */
    ret = mapping_switch(MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* save origin value */
    ret = fts_test_read_reg(FACTORY_REG_NORMALIZE, &normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read normalize fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_FIR, &fir);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0xFB error,ret=%d\n", ret);
        goto test_err;
    }

    /* set to auto normalize */
    if (tdata->normalize != normalize) {
        ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, tdata->normalize);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("write normalize fail,ret=%d\n", ret);
            goto restore_reg;
        }
    }

    result = true;
    result2 = true;
    if (NORMALIZE_AUTO == tdata->normalize) {
        FTS_TEST_SAVE_INFO( "NORMALIZE_AUTO:\n");
        rawdata_tmp = rawdata + rawdata_cnt;
        ret = get_rawdata_mc(0x81, 0x01, rawdata_tmp);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("get rawdata fail,ret=%d\n", ret);
            goto restore_reg;
        }

        show_data(rawdata_tmp, false);

        /* compare */
        result = compare_array(rawdata_tmp,
                               thr->rawdata_h_min,
                               thr->rawdata_h_max,
                               false);

        rawdata_cnt += tdata->node.node_num;
    } else if (NORMALIZE_OVERALL == tdata->normalize) {
        if (thr->basic.rawdata_set_lfreq) {
            FTS_TEST_SAVE_INFO( "NORMALIZE_OVERALL + SET_LOW_FREQ:\n");
            rawdata_tmp = rawdata + rawdata_cnt;
            ret = get_rawdata_mc(0x80, 0x00, rawdata_tmp);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("get rawdata fail,ret=%d\n", ret);
                goto restore_reg;
            }

            show_data(rawdata_tmp, false);

            /* compare */
            result = compare_array(rawdata_tmp,
                                   thr->rawdata_l_min,
                                   thr->rawdata_l_max,
                                   false);

            rawdata_cnt += tdata->node.node_num;
        }

        if (thr->basic.rawdata_set_hfreq) {
            FTS_TEST_SAVE_INFO( "NORMALIZE_OVERALL + SET_HIGH_FREQ:\n");
            rawdata_tmp = rawdata + rawdata_cnt;
            ret = get_rawdata_mc(0x81, 0x00, rawdata_tmp);
            if (ret < 0) {
                FTS_TEST_SAVE_ERR("get rawdata fail,ret=%d\n", ret);
                goto restore_reg;
            }

            show_data(rawdata_tmp, false);

            /* compare */
            result2 = compare_array(rawdata_tmp,
                                    thr->rawdata_h_min,
                                    thr->rawdata_h_max,
                                    false);

            rawdata_cnt += tdata->node.node_num;
        }
    }

restore_reg:
    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore normalize fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FIR, fir);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0xFB fail,ret=%d\n", ret);
    }

test_err:
    if (result && result2) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("------ Rawdata Test PASS\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_INFO("------ Rawdata Test NG\n");
    }

    /* save data */
    fts_test_save_data("Rawdata Test", CODE_M_RAWDATA_TEST,
                       rawdata, rawdata_cnt, false, false, *test_result);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft5822_scap_cb_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    bool tmp2_result = false;
    u8 wc_sel = 0;
    u8 sc_mode = 0;
    int byte_num = 0;
    bool fw_wp_check = false;
    bool tx_check = false;
    bool rx_check = false;
    int *scap_cb = NULL;
    int *scb_tmp = NULL;
    int scb_cnt = 0;
    struct mc_sc_threshold *thr = &tdata->ic.mc_sc.thr;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("\n============ Test Item: Scap CB Test\n");
    memset(tdata->buffer, 0, tdata->buffer_length);
    scap_cb = tdata->buffer;
    byte_num = tdata->sc_node.node_num;

    if ((tdata->sc_node.node_num * 2) > tdata->buffer_length) {
        FTS_TEST_SAVE_ERR("scap cb num(%d) > buffer length(%d)",
                          tdata->sc_node.node_num * 2,
                          tdata->buffer_length);
        ret = -EINVAL;
        goto test_err;
    }

    if (!thr->scap_cb_on_min || !thr->scap_cb_on_max
        || !thr->scap_cb_off_min || !thr->scap_cb_off_max || !test_result) {
        FTS_TEST_SAVE_ERR("scap_cb_on/off_min/max test_result is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    /* SCAP CB is in no-mapping mode */
    ret = mapping_switch(NO_MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch no-mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* get waterproof channel select */
    ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read water_channel_sel fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_MC_SC_MODE, &sc_mode);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read sc_mode fail,ret=%d\n", ret);
        goto test_err;
    }

    /* water proof on check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);
    if (thr->basic.scap_cb_wp_on_check && fw_wp_check) {
        scb_tmp = scap_cb + scb_cnt;
        /* 1:waterproof 0:non-waterproof */
        ret = get_cb_mc_sc(WATER_PROOF_ON, byte_num, scb_tmp, DATA_ONE_BYTE);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
            goto restore_reg;
        }

        /* show & save Scap CB */
        FTS_TEST_SAVE_INFO("scap_cb in waterproof on mode:\n");
        show_data_mc_sc(scb_tmp);

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_RX);
        tmp_result = compare_mc_sc(tx_check, rx_check, scb_tmp,
                                   thr->scap_cb_on_min,
                                   thr->scap_cb_on_max);

        scb_cnt += tdata->sc_node.node_num;
    } else {
        tmp_result = true;
    }

    /* water proof off check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);
    if (thr->basic.scap_cb_wp_off_check && fw_wp_check) {
        scb_tmp = scap_cb + scb_cnt;
        /* 1:waterproof 0:non-waterproof */
        ret = get_cb_mc_sc(WATER_PROOF_OFF, byte_num, scb_tmp, DATA_ONE_BYTE);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("read sc_cb fail,ret=%d\n", ret);
            goto restore_reg;
        }

        /* show & save Scap CB */
        FTS_TEST_SAVE_INFO("scap_cb in waterproof off mode:\n");
        show_data_mc_sc(scb_tmp);

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_RX);
        tmp2_result = compare_mc_sc(tx_check, rx_check, scb_tmp,
                                    thr->scap_cb_off_min,
                                    thr->scap_cb_off_max);

        scb_cnt += tdata->sc_node.node_num;
    } else {
        tmp2_result = true;
    }

restore_reg:
    ret = fts_test_write_reg(FACTORY_REG_MC_SC_MODE, sc_mode);/* set the origin value */
    if (ret) {
        FTS_TEST_SAVE_ERR("write sc mode fail,ret=%d\n", ret);
    }
test_err:
    if (tmp_result && tmp2_result) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("------ SCAP CB Test PASS\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_ERR("------ SCAP CB Test NG\n");
    }

    /* save data */
    fts_test_save_data("SCAP CB Test", CODE_M_SCAP_CB_TEST,
                       scap_cb, scb_cnt, true, false, *test_result);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft5822_scap_rawdata_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    bool tmp2_result = false;
    u8 wc_sel = 0;
    bool fw_wp_check = false;
    bool tx_check = false;
    bool rx_check = false;
    int *scap_rawdata = NULL;
    int *srawdata_tmp = NULL;
    int srawdata_cnt = 0;
    struct mc_sc_threshold *thr = &tdata->ic.mc_sc.thr;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("\n============ Test Item: Scap Rawdata Test\n");
    memset(tdata->buffer, 0, tdata->buffer_length);
    scap_rawdata = tdata->buffer;

    if ((tdata->sc_node.node_num * 2) > tdata->buffer_length) {
        FTS_TEST_SAVE_ERR("scap rawdata num(%d) > buffer length(%d)",
                          tdata->sc_node.node_num * 2,
                          tdata->buffer_length);
        ret = -EINVAL;
        goto test_err;
    }

    if (!thr->scap_rawdata_on_min || !thr->scap_rawdata_on_max
        || !thr->scap_rawdata_off_min || !thr->scap_rawdata_off_max
        || !test_result) {
        FTS_TEST_SAVE_ERR("scap_rawdata_on/off_min/max test_result is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("enter factory mode fail,ret=%d\n", ret);
        goto test_err;
    }

    /* SCAP CB is in no-mapping mode */
    ret = mapping_switch(NO_MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch no-mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* get waterproof channel select */
    ret = fts_test_read_reg(FACTORY_REG_WC_SEL, &wc_sel);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read water_channel_sel fail,ret=%d\n", ret);
        goto test_err;
    }

    /* scan rawdata */
    ret = start_scan();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("scan scap rawdata fail\n");
        goto test_err;
    }

    /* water proof on check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_ON);
    if (thr->basic.scap_rawdata_wp_on_check && fw_wp_check) {
        srawdata_tmp = scap_rawdata + srawdata_cnt;
        ret = get_rawdata_mc_sc(WATER_PROOF_ON, srawdata_tmp);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("get scap(WP_ON) rawdata fail\n");
            goto test_err;
        }

        FTS_TEST_SAVE_INFO("scap_rawdata in waterproof on mode:\n");
        show_data_mc_sc(srawdata_tmp);

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_ON_RX);
        tmp_result = compare_mc_sc(tx_check, rx_check, srawdata_tmp,
                                   thr->scap_rawdata_on_min,
                                   thr->scap_rawdata_on_max);

        srawdata_cnt += tdata->sc_node.node_num;
    } else {
        tmp_result = true;
    }

    /* water proof off check */
    fw_wp_check = get_fw_wp(wc_sel, WATER_PROOF_OFF);
    if (thr->basic.scap_rawdata_wp_off_check && fw_wp_check) {
        srawdata_tmp = scap_rawdata + srawdata_cnt;
        ret = get_rawdata_mc_sc(WATER_PROOF_OFF, srawdata_tmp);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("get scap(WP_OFF) rawdata fail\n");
            goto test_err;
        }

        FTS_TEST_SAVE_INFO("scap_rawdata in waterproof off mode:\n");
        show_data_mc_sc(srawdata_tmp);

        /* compare */
        tx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_TX);
        rx_check = get_fw_wp(wc_sel, WATER_PROOF_OFF_RX);
        tmp2_result = compare_mc_sc(tx_check, rx_check, srawdata_tmp,
                                    thr->scap_rawdata_off_min,
                                    thr->scap_rawdata_off_max);

        srawdata_cnt += tdata->sc_node.node_num;
    } else {
        tmp2_result = true;
    }

test_err:
    if (tmp_result && tmp2_result) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("------ SCAP Rawdata Test PASS\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_INFO("------ SCAP Rawdata Test NG\n");
    }

    /* save data */
    fts_test_save_data("SCAP Rawdata Test", CODE_M_SCAP_RAWDATA_TEST,
                       scap_rawdata, srawdata_cnt, true, false, *test_result);


    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft5822_panel_differ_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    bool tmp_result = false;
    int i = 0;
    u8 fre = 0;
    u8 fir = 0;
    u8 normalize = 0;
    int *panel_differ = NULL;
    struct mc_sc_threshold *thr = &tdata->ic.mc_sc.thr;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("\n============ Test Item: Panel Differ Test\n");
    memset(tdata->buffer, 0, tdata->buffer_length);
    panel_differ = tdata->buffer;

    if (!thr->panel_differ_min || !thr->panel_differ_max || !test_result) {
        FTS_TEST_SAVE_ERR("panel_differ_h_min/max test_result is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        goto test_err;
    }

    /* panel differ test in mapping mode */
    ret = mapping_switch(MAPPING);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("switch mapping fail,ret=%d\n", ret);
        goto test_err;
    }

    /* save origin value */
    ret = fts_test_read_reg(FACTORY_REG_NORMALIZE, &normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read normalize fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_FIR, &fir);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0xFB fail,ret=%d\n", ret);
        goto test_err;
    }

    /* set to overall normalize */
    if (normalize != NORMALIZE_OVERALL) {
        ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, NORMALIZE_OVERALL);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("write normalize fail,ret=%d\n", ret);
            goto restore_reg;
        }
    }

    ret = get_rawdata_mc(0x81, 0x00, panel_differ);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get differ fail,ret=%d\n", ret);
        goto restore_reg;
    }

    for (i = 0; i < tdata->node.node_num; i++) {
        panel_differ[i] = panel_differ[i] / 10;
    }

    /* show test data */
    show_data(panel_differ, false);

    /* compare */
    tmp_result = compare_array(panel_differ,
                               thr->panel_differ_min,
                               thr->panel_differ_max,
                               false);

restore_reg:
    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore normalize fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FIR, fir);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0xFB fail,ret=%d\n", ret);
    }
test_err:
    /* result */
    if (tmp_result) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("------ Panel Diff Test PASS\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_ERR("------ Panel Diff Test NG\n");
    }

    /* save test data */
    fts_test_save_data("Panel Diff Test", CODE_M_PANELDIFFER_TEST,
                       panel_differ, 0, false, false, *test_result);

    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int ft5822_uniformity_test(struct fts_test *tdata, bool *test_result)
{
    int ret = 0;
    int index = 0;
    int row = 0;
    int col = 1;
    int i = 0;
    int deviation = 0;
    int max = 0;
    int min = 0;
    int uniform = 0;
    int *rawdata = NULL;
    int *rawdata_linearity = NULL;
    int *rl_tmp = NULL;
    int rl_cnt = 0;
    int offset = 0;
    int offset2 = 0;
    int tx_num = 0;
    int rx_num = 0;
    u8 fre = 0;
    u8 fir = 0;
    u8 normalize = 0;
    struct mc_sc_threshold *thr = &fts_ftest->ic.mc_sc.thr;
    bool result = false;
    bool result2 = false;
    bool result3 = false;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_SAVE_INFO("\n============ Test Item: rawdata unfiormity test\n");
    memset(tdata->buffer, 0, tdata->buffer_length);
    rawdata = tdata->buffer;
    tx_num = tdata->node.tx_num;
    rx_num = tdata->node.rx_num;

    if (!thr->tx_linearity_max || !thr->rx_linearity_max
        || !tdata->node_valid) {
        FTS_TEST_SAVE_ERR("tx/rx_lmax/node_valid is null\n");
        ret = -EINVAL;
        goto test_err;
    }

    ret = enter_factory_mode();
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("failed to enter factory mode,ret=%d\n", ret);
        goto test_err;
    }

    rawdata_linearity = fts_malloc(tdata->node.node_num * 2 * sizeof(int));
    if (!rawdata_linearity) {
        FTS_TEST_SAVE_ERR("rawdata_linearity buffer malloc fail");
        ret = -ENOMEM;
        goto test_err;
    }

    /* rawdata unfiormity test in mapping mode */
    ret = mapping_switch(MAPPING);
    if (ret) {
        FTS_TEST_SAVE_ERR("failed to switch_to_mapping,ret=%d", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_NORMALIZE, &normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("read normalize fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_FRE_LIST, &fre);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0x0A fail,ret=%d\n", ret);
        goto test_err;
    }

    ret = fts_test_read_reg(FACTORY_REG_FIR, &fir);
    if (ret) {
        FTS_TEST_SAVE_ERR("read 0xFB error,ret=%d\n", ret);
        goto test_err;
    }

    if (normalize != 0x01) {
        ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, 0x01);
        if (ret < 0) {
            FTS_TEST_SAVE_ERR("write normalize fail,ret=%d\n", ret);
            goto restore_reg;
        }
    }

    /* set frequecy high */
    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, 0x81);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set frequecy fail,ret=%d\n", ret);
        goto restore_reg;
    }
    sys_delay(10);

    /* fir enable */
    ret = fts_test_write_reg(FACTORY_REG_FIR, 1);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("set fir fail,ret=%d\n", ret);
        goto restore_reg;
    }
    sys_delay(10);

    /* change register value before,need to lose 3 frame data */
    for (index = 0; index < 3; ++index) {
        ret = get_rawdata(rawdata);
    }
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("get rawdata fail,ret=%d\n", ret);
        goto restore_reg;
    }
    print_buffer(rawdata, tdata->node.node_num, tdata->node.rx_num);

    result = true;
    if (thr->basic.uniformity_check_tx) {
        FTS_TEST_SAVE_INFO("Check Tx Linearity\n");
        rl_tmp = rawdata_linearity + rl_cnt;
        for (row = 0; row < tx_num; row++) {
            for (col = 1; col <  rx_num; col++) {
                offset = row * rx_num + col;
                offset2 = row * rx_num + col - 1;
                deviation = abs( rawdata[offset] - rawdata[offset2]);
                max = max(rawdata[offset], rawdata[offset2]);
                max = max ? max : 1;
                rl_tmp[offset] = 100 * deviation / max;
            }
        }
        /*show data in result.txt*/
        FTS_TEST_SAVE_INFO(" Tx Linearity:\n");
        show_data(rl_tmp, false);
        FTS_TEST_SAVE_INFO("\n" );

        /* compare */
        result = compare_array(rl_tmp,
                               thr->tx_linearity_min,
                               thr->tx_linearity_max,
                               false);

        rl_cnt += tdata->node.node_num;
    }

    result2 = true;
    if (thr->basic.uniformity_check_rx) {
        FTS_TEST_SAVE_INFO("Check Rx Linearity\n");
        rl_tmp = rawdata_linearity + rl_cnt;
        for (row = 1; row < tx_num; row++) {
            for (col = 0; col < rx_num; col++) {
                offset = row * rx_num + col;
                offset2 = (row - 1) * rx_num + col;
                deviation = abs(rawdata[offset] - rawdata[offset2]);
                max = max(rawdata[offset], rawdata[offset2]);
                max = max ? max : 1;
                rl_tmp[offset] = 100 * deviation / max;
            }
        }

        FTS_TEST_SAVE_INFO("Rx Linearity:\n");
        show_data(rl_tmp, false);
        FTS_TEST_SAVE_INFO("\n");

        /* compare */
        result2 = compare_array(rl_tmp,
                                thr->rx_linearity_min,
                                thr->rx_linearity_max,
                                false);
        rl_cnt += tdata->node.node_num;
    }

    result3 = true;
    if (thr->basic.uniformity_check_min_max) {
        FTS_TEST_SAVE_INFO("Check Min/Max\n") ;
        min = 100000;
        max = -100000;
        for (i = 0; i < tdata->node.node_num; i++) {
            if (0 == tdata->node_valid[i])
                continue;
            min = min(min, rawdata[i]);
            max = max(max, rawdata[i]);
        }
        max = !max ? 1 : max;
        uniform = 100 * abs(min) / abs(max);

        FTS_TEST_SAVE_INFO("min:%d, max:%d, get value of min/max:%d\n",
                           min, max, uniform);
        if (uniform < thr->basic.uniformity_min_max_hole) {
            result3 = false;
            FTS_TEST_SAVE_ERR("min_max out of range, set value: %d\n",
                              thr->basic.uniformity_min_max_hole);
        }
    }

restore_reg:
    /* set the origin value */
    ret = fts_test_write_reg(FACTORY_REG_NORMALIZE, normalize);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore normalize fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FRE_LIST, fre);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0x0A fail,ret=%d\n", ret);
    }

    ret = fts_test_write_reg(FACTORY_REG_FIR, fir);
    if (ret < 0) {
        FTS_TEST_SAVE_ERR("restore 0xFB fail,ret=%d\n", ret);
    }

test_err:
    if (result && result2 && result3) {
        *test_result = true;
        FTS_TEST_SAVE_INFO("uniformity test is OK\n");
    } else {
        *test_result = false;
        FTS_TEST_SAVE_ERR("uniformity test is NG\n");
    }

    fts_test_save_data("Rawdata Uniformity Test",
                       CODE_M_RAWDATA_UNIFORMITY_TEST,
                       rawdata_linearity, tdata->node.node_num * 2, false, false, *test_result);

    fts_free(rawdata_linearity);
    FTS_TEST_FUNC_EXIT();
    return ret;
}

static int start_test_ft5822(void)
{
    int ret = 0;
    struct fts_test *tdata = fts_ftest;
    struct mc_sc_testitem *test_item = &tdata->ic.mc_sc.u.item;
    bool temp_result = false;
    bool test_result = true;

    FTS_TEST_FUNC_ENTER();
    FTS_TEST_INFO("test item:0x%x", fts_ftest->ic.mc_sc.u.tmp);

    /* rawdata test */
    if (true == test_item->rawdata_test) {
        ret = ft5822_rawdata_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    if (true == test_item->rawdata_uniformity_test) {
        ret = ft5822_uniformity_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    /* scap_cb test */
    if (true == test_item->scap_cb_test) {
        ret = ft5822_scap_cb_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    /* scap_rawdata test */
    if (true == test_item->scap_rawdata_test) {
        ret = ft5822_scap_rawdata_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    /* panel differ test */
    if (true == test_item->panel_differ_test) {
        ret = ft5822_panel_differ_test(tdata, &temp_result);
        if ((ret < 0) || (false == temp_result)) {
            test_result = false;
        }
    }

    /* restore mapping state */
    fts_test_write_reg(FACTORY_REG_NOMAPPING, tdata->mapping);
    return test_result;
}

struct test_funcs test_func_ft5822 = {
    .ctype = {0x01,0x86},
    .hwtype = IC_HW_MC_SC,
    .key_num_total = 0,
    .rawdata2_support = true,
    .start_test = start_test_ft5822,
};

