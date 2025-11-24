//#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#include "fpga_vir_cam.h"


/*FPGA START/STOP TRANS REG*/
#define FPGA_VIR_CAM_REG_START ((__u16)0x0) // 


#define FPGA_VIR_CAM_PROTECT_REG ((__u16)0x10)


#define FPGA_VIR_CAM_DATA_LANES 4

struct fpga_vir_cam_mode {
    __u32 width;
    __u32 height;
    __u32 link_freq_index;
    __u32 pixel_rate;
    __u32 pixel_code;
};

struct fpga_vir_cam {  
    struct v4l2_subdev sd;
    struct media_pad pad;

    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *link_freq;
    struct v4l2_ctrl *pixel_rate;
    struct v4l2_ctrl *rd_reg;
    struct v4l2_ctrl *wr_reg;
   
    struct fpga_vir_cam_mode *cur_mode;

    struct mutex mutex;
    int streaming;
    int identified;
    
    /* Other members as needed */
};

enum link_freq_menu_index {
    FPGA_VIR_CAM_LINK_FREQ_1250_MHZ = 0,     
};

static const s64 link_freq_menu_items[] = {
    1250000000ULL,
};

static const struct fpga_vir_cam_mode supported_modes[] = {
    {
        .width = 1024,
        .height = 1025,
        .link_freq_index = 0,
        .pixel_rate = 742500000,
        .pixel_code = MEDIA_BUS_FMT_Y8_1X8,
    },
    {
        .width = 1024,
        .height = 513,
        .link_freq_index = 0,
        .pixel_rate = 371250000,
        .pixel_code = MEDIA_BUS_FMT_Y8_1X8,
    },
    {
        .width = 1024,
        .height = 513,
        .link_freq_index = 0,
        .pixel_rate = 371250000,
        .pixel_code = MEDIA_BUS_FMT_Y8_1X8,
    },
};

#define to_fpga_vir_cam(_sd)	container_of(_sd, struct fpga_vir_cam, sd)

static int fpga_vir_cam_read_reg(struct fpga_vir_cam *fvc,
            __u16 reg, __u16 *val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&fvc->sd);
    int ret;
    struct i2c_msg msgs[2];
    u8 reg_buf[2];
    __u16 val_be;

    reg_buf[0] = reg >> 8;
    reg_buf[1] = reg & 0xFF;
 
    msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = reg_buf;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = 2;
    msgs[1].buf = (u8 *)&val_be;

    mutex_lock(&fvc->mutex);
    ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    mutex_unlock(&fvc->mutex);

    if (ret != ARRAY_SIZE(msgs))
		return -EIO;
    *val = be16_to_cpu(val_be);
    return 0;
}
static int fpga_vir_cam_write_reg(struct fpga_vir_cam *fvc,
            __u16 reg, __u16 val)
{
    struct i2c_client *client = v4l2_get_subdevdata(&fvc->sd);
    int ret;
    u8 buf[4];
    buf[0] = reg >> 8;
    buf[1] = reg & 0xFF;
    buf[2] = (val) >> 8;
    buf[3] = (val) & 0xFF;
    mutex_lock(&fvc->mutex);
    ret = i2c_master_send(client, buf, 4);
    mutex_unlock(&fvc->mutex);
    if (ret != 4)
        return -EIO;
    return 0;
}

static int fpga_vir_cam_read_ctrl(struct v4l2_ctrl *ctrl)
{
    struct fpga_vir_cam *dev = container_of(ctrl->handler, 
                                        struct fpga_vir_cam, ctrl_handler);
    int ret = 0;
    struct fpga_vir_cam_reg *reg = (struct fpga_vir_cam_reg *)ctrl->p_new.p;
    
    switch (ctrl->id) {
    case FPGA_VIR_CAM_V4L2_REG_READ:
        ret = fpga_vir_cam_read_reg(dev, reg->reg_addr, &reg->reg_value);
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}

static int fpga_vir_cam_write_ctrl(struct v4l2_ctrl *ctrl)
{
    struct fpga_vir_cam *fvc = container_of(ctrl->handler, 
                                        struct fpga_vir_cam, ctrl_handler);
    int ret = 0;
    if(fvc->streaming) {
        return -EBUSY;
    }
    struct fpga_vir_cam_reg *reg = (struct fpga_vir_cam_reg *)ctrl->p_new.p;
    switch (ctrl->id) {
    case FPGA_VIR_CAM_V4L2_REG_WRITE:
        if (reg->reg_addr < FPGA_VIR_CAM_PROTECT_REG) {
            ret = -EACCES;
            break;
        }
        ret = fpga_vir_cam_write_reg(fvc, reg->reg_addr, reg->reg_value);
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}
static const struct v4l2_ctrl_ops fpga_vir_cam_ctrl_ops = {
    .g_volatile_ctrl = fpga_vir_cam_read_ctrl,
    .s_ctrl = fpga_vir_cam_write_ctrl,
};

static int fpga_vir_cam_set_stream(struct v4l2_subdev *sd, int enable)
{
    struct fpga_vir_cam *fvc = to_fpga_vir_cam(sd);
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;

    mutex_lock(&fvc->mutex);
    if (fvc->streaming == enable) {
        mutex_unlock(&fvc->mutex);
        return 0;
    }

    if (enable) {
        ret = pm_runtime_resume_and_get(&client->dev);
        if (ret < 0)
            goto err_unlock;

        ret = fpga_vir_cam_write_reg(fvc, FPGA_VIR_CAM_REG_START, 1);
        if (ret) {
            dev_err(&client->dev, "failed to start streaming\n");
            goto err_rpm_put;
        }
    } else {
        ret = fpga_vir_cam_write_reg(fvc, FPGA_VIR_CAM_REG_START, 0);
        if (ret) {
            dev_err(&client->dev, "failed to stop streaming\n");
            goto err_unlock;
        }
        pm_runtime_put(&client->dev);
    }

    fvc->streaming = enable;
    mutex_unlock(&fvc->mutex);

    return ret;
}

static const struct v4l2_subdev_video_ops fpga_vir_cam_stream_ops = {
    .s_stream = fpga_vir_cam_set_stream,
};

static int fpga_vir_cam_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	/* Only one bayer order(GRBG) is supported */
	if (code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_Y8_1X8;

	return 0;
}

static int fpga_vir_cam_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_Y8_1X8)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static void fpga_vir_cam_update_pad_format(const struct fpga_vir_cam_mode *mode,
				      struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = MEDIA_BUS_FMT_Y8_1X8;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int fpga_vir_cam_do_get_pad_format(struct fpga_vir_cam *fvc,
				     struct v4l2_subdev_state *sd_state,
				     struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt;
	struct v4l2_subdev *sd = &fvc->sd;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
		fmt->format = *framefmt;
	} else {
		fpga_vir_cam_update_pad_format(fvc->cur_mode, fmt);
	}

	return 0;
}

static int fpga_vir_cam_get_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *fmt)
{
	struct fpga_vir_cam *fvc = to_fpga_vir_cam(sd);
	int ret;

	mutex_lock(&fvc->mutex);
	ret = fpga_vir_cam_do_get_pad_format(fvc, sd_state, fmt);
	mutex_unlock(&fvc->mutex);

	return ret;
}

static int fpga_vir_cam_set_pad_format(struct v4l2_subdev *sd,
		       struct v4l2_subdev_state *sd_state,
		       struct v4l2_subdev_format *fmt)
{
    struct fpga_vir_cam *fvc = to_fpga_vir_cam(sd);
    const struct fpga_vir_cam_mode *mode;
    struct v4l2_mbus_framefmt *framefmt;
    s32 vblank_def;
    s32 vblank_min;
    s64 h_blank;
    s64 pixel_rate;
    s64 link_freq;

    mutex_lock(&fvc->mutex);

    /* Only one raw bayer(Y8) order is supported */
    if (fmt->format.code != MEDIA_BUS_FMT_Y8_1X8)
        fmt->format.code = MEDIA_BUS_FMT_Y8_1X8;

    mode = v4l2_find_nearest_size(supported_modes,
                      ARRAY_SIZE(supported_modes),
                      width, height,
                      fmt->format.width, fmt->format.height);
    fpga_vir_cam_update_pad_format(mode, fmt);
    if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
        framefmt = v4l2_subdev_get_try_format(sd, sd_state, fmt->pad);
        *framefmt = fmt->format;
    } else {
        fvc->cur_mode = mode;
        __v4l2_ctrl_s_ctrl(fvc->link_freq, mode->link_freq_index);
        link_freq = link_freq_menu_items[mode->link_freq_index];
        pixel_rate = link_freq_to_pixel_rate(link_freq);
        __v4l2_ctrl_s_ctrl_int64(fvc->pixel_rate, pixel_rate);

        /* Update limits and set FPS to default */
        vblank_def = fvc->cur_mode->vts_def -
                 fvc->cur_mode->height;
        vblank_min = fvc->cur_mode->vts_min -
                 fvc->cur_mode->height;
        __v4l2_ctrl_modify_range(fvc->vblank, vblank_min,
                     FPGA_VIR_CAM_VTS_MAX
                     - fvc->cur_mode->height,
                     1,
                     vblank_def);
        __v4l2_ctrl_s_ctrl(fvc->vblank, vblank_def);
        h_blank =
            link_freq_configs[mode->link_freq_index].pixels_per_line
             - fvc->cur_mode->width;
        __v4l2_ctrl_modify_range(fvc->hblank, h_blank,
                     h_blank, 1, h_blank);
    }

    
}

static const struct v4l2_subdev_pad_ops fpga_vir_cam_pad_ops = {
	.enum_mbus_code = fpga_vir_cam_enum_mbus_code,
	.get_fmt = fpga_vir_cam_get_pad_format,
	.set_fmt = fpga_vir_cam_set_pad_format,
	.enum_frame_size = fpga_vir_cam_enum_frame_size,
};

static const struct v4l2_subdev_ops fpga_vir_cam_subdev_ops = {
	.video = &fpga_vir_cam_stream_ops,
	.pad = &fpga_vir_cam_pad_ops,
};

static int fpga_vir_cam_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
    /*set defualt mode*/
    const struct fpga_vir_cam_mode *default_mode = &supported_modes[0];
	struct fpga_vir_cam *fvc = to_fpga_vir_cam(sd);
	struct v4l2_mbus_framefmt *try_fmt = v4l2_subdev_get_try_format(sd,
									fh->state,
									0);

	mutex_lock(&fvc->mutex);

	/* Initialize try_fmt */
	try_fmt->width = default_mode->width;
	try_fmt->height = default_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	/* No crop or compose */
	mutex_unlock(&fvc->mutex);
    return 0;
}
static const struct v4l2_subdev_internal_ops fpga_vir_cam_internal_ops = {
    .open = fpga_vir_cam_open,
};

static int fpga_vir_cam_init_controls(struct fpga_vir_cam *fvc)
{
   
    struct v4l2_ctrl_handler *ctrl_hdlr;
    struct v4l2_ctrl_config reg_read_cfg = {0};
    struct v4l2_ctrl_config reg_write_cfg = {0};
    struct i2c_client *client = v4l2_get_subdevdata(&fvc->sd);
    int ret;
    __u32 max, pixel_rate_max, pixel_rate_min ;

    ctrl_hdlr = &fvc->ctrl_handler;
    ret = v4l2_ctrl_handler_init(ctrl_hdlr, 4);
    if (IS_ERR(ret))
        return ret;

    mutex_init(&fvc->mutex);
    ctrl_hdlr->lock = &fvc->mutex;

    max = ARRAY_SIZE(link_freq_menu_items) - 1;
    fvc->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr,
                        &fpga_vir_cam_ctrl_ops,
                        V4L2_CID_LINK_FREQ,
                        max,
                        0,
                        link_freq_menu_items);
	if (fvc->link_freq)
		fvc->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    pixel_rate_max = link_freq_to_pixel_rate(link_freq_menu_items[0]);
	pixel_rate_min = 0;
	/* By default, PIXEL_RATE is read only */
	fvc->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &fpga_vir_cam_ctrl_ops,
					      V4L2_CID_PIXEL_RATE,
					      pixel_rate_min, pixel_rate_max,
					      1, pixel_rate_max);

    reg_read_cfg.ops = &fpga_vir_cam_ctrl_ops;
    reg_read_cfg.id = FPGA_VIR_CAM_V4L2_REG_READ;
    reg_read_cfg.name = "Register Read";
    reg_read_cfg.type = V4L2_CTRL_TYPE_INTEGER;
    reg_read_cfg.flags = V4L2_CTRL_FLAG_VOLATILE | 
                        V4L2_CTRL_FLAG_READ_ONLY |
                        V4L2_CTRL_FLAG_HAS_PAYLOAD;
    reg_read_cfg.min = 0;
    reg_read_cfg.max = 0xFFFFFFFF;
    reg_read_cfg.step = 1;
    reg_read_cfg.def = 0;
    reg_read_cfg.dims[0] = sizeof(struct fpga_vir_cam_reg);
    reg_read_cfg.elem_size = sizeof(struct fpga_vir_cam_reg);
    fvc->rd_reg =  v4l2_ctrl_new_custom(&fvc->ctrl_handler, &reg_read_cfg, NULL);

    reg_write_cfg.ops = &fpga_vir_cam_ctrl_ops;
    reg_write_cfg.id =  FPGA_VIR_CAM_V4L2_REG_WRITE;
    reg_write_cfg.name = "Register Write";
    reg_write_cfg.type = V4L2_CTRL_TYPE_INTEGER;
    reg_write_cfg.flags = V4L2_CTRL_FLAG_WRITE_ONLY |
                         V4L2_CTRL_FLAG_HAS_PAYLOAD;
    reg_write_cfg.min = 0;
    reg_write_cfg.max = 0xFFFFFFFF;
    reg_write_cfg.step = 1;
    reg_write_cfg.def = 0;
    reg_write_cfg.dims[0] = sizeof(struct fpga_vir_cam_reg);
    reg_write_cfg.elem_size = sizeof(struct fpga_vir_cam_reg);
    
    fvc->wr_reg = v4l2_ctrl_new_custom(&fvc->ctrl_handler, &reg_write_cfg, NULL);
    
   
    if (ctrl_hdlr->error) {
        ret = ctrl_hdlr->error;
        dev_err(&client->dev, "error while initializing control handler: %d\n", ret);
        v4l2_ctrl_handler_free(&fvc->ctrl_handler);
        return ret;
    }

    // fvc->rd_reg = v4l2_ctrl_new_std(ctrl_hdlr, &fpga_vir_cam_ctrl_ops,
    //                                 V4L2_CID_PRIVATE_BASE + 0,
    //                                 0, 0xFFFFFFFF, 1, 0);
    // fvc->wr_reg = v4l2_ctrl_new_std(ctrl_hdlr, &fpga_vir_cam_ctrl_ops,
    //                                 V4L2_CID_PRIVATE_BASE + 1,
    //                                 0, 0xFFFFFFFF, 1, 0);
    fvc->sd.ctrl_handler = ctrl_hdlr;

    return 0;
}

static int fpga_vir_cam_free_controls(struct fpga_vir_cam *fvc)
{
    v4l2_ctrl_handler_free(&fvc->ctrl_handler);
    return 0;
}

static int fpga_vir_cam_chkeck_hwcfg(struct device *dev)
{
    struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *ep;
	struct fwnode_handle *fwnode = dev_fwnode(dev);

    if (!fwnode)
        return -ENXIO;

    ep = fwnode_graph_get_next_endpoint(fwnode, NULL);
	if (!ep)
		return -ENXIO;

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (IS_ERR(ret))
		return ret;

    if (bus_cfg.bus.mipi_csi2.num_data_lanes != FPGA_VIR_CAM_DATA_LANES) {
    dev_err(dev, "number of CSI2 data lanes %d is not supported",
        bus_cfg.bus.mipi_csi2.num_data_lanes);
    ret = -EINVAL;
    goto out_err;
	}

	if (!bus_cfg.nr_of_link_frequencies) {
		dev_err(dev, "no link frequencies defined");
		ret = -EINVAL;
		goto out_err;
	}

out_err:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}
static int fpga_vir_cam_probe(struct i2c_client *client)
{
    struct fpga_vir_cam *fvc;
    int ret;

    fvc = devm_kzalloc(&client->dev, sizeof(*fvc), GFP_KERNEL);
    if (!fvc)
        return -ENOMEM;

    ret = fpga_vir_cam_chkeck_hwcfg(&client->dev);
    if (IS_ERR(ret)) {
        dev_err(&client->dev, "failed to check hwcfg: %d", ret);  
        return ret;
    }
    v4l2_i2c_subdev_init(&fvc->sd, client, &fpga_vir_cam_stream_ops);

    mutex_init(&fvc->mutex);
    fvc->streaming = false;
    ret = fpga_vir_cam_init_controls(fvc);
    if (IS_ERR(ret)) {
        dev_err(&client->dev, "failed to init controls: %d\n", ret);
        return ret;
    }

    /* Initialize subdev */
	fvc->sd.internal_ops = &fpga_vir_cam_internal_ops;
	fvc->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	fvc->sd.entity.ops = &fpga_vir_cam_subdev_entity_ops;
	fvc->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pad */
	fvc->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&fvc->sd.entity, 1, &fvc->pad);
	if (ret) {
		dev_err(&client->dev, "%s failed:%d\n", __func__, ret);
		goto error_handler_free;
	}


    ret = v4l2_async_register_subdev(&fvc->sd);
    if (ret)
        goto err_free_ctrls;

    return ret;

err_free_ctrls:
    fpga_vir_cam_free_controls(fvc);
    mutex_destroy(&fvc->mutex);
    return ret;
}

static void fpga_vir_cam_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct fpga_vir_cam *fvc = to_fpga_vir_cam(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	fpga_vir_cam_free_controls(fvc);
	//pm_runtime_disable(&client->dev);
}

static struct i2c_driver fpga_vir_cam_i2c_driver = {
	.driver = {
		.name = "fpga_vir_cam",
		//.pm = &fpga_vir_cam_pm_ops,
	},
	.probe_new = fpga_vir_cam_probe,
	.remove = fpga_vir_cam_remove,
	//.flags = I2C_DRV_ACPI_WAIVE_D0_PROBE,
};

module_i2c_driver(fpga_vir_cam_i2c_driver);

MODULE_DESCRIPTION("FPGA Virtual Camera Driver");
MODULE_AUTHOR("YangGP <1945728545@qq.com>");
MODULE_LICENSE("GPL");