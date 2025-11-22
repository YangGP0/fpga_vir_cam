#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

/*v4l2 ctrl id*/ 
#define FPGA_VIR_CAM_V4L2_REG_READ  (V4L2_CID_PRIVATE_BASE + 0)
#define FPGA_VIR_CAM_V4L2_REG_WRITE (V4L2_CID_PRIVATE_BASE + 1)
/*FPGA START/STOP TRANS REG*/
#define FPGA_VIR_CAM_REG_START ((__u16)0x0) // 


#define FPGA_VIR_CAM_PROTECT_REG ((__u16)0x10)

struct fpga_vir_cam_reg {
    __u16 reg_value;
    __u16 reg_addr;
};

struct fpga_vir_cam {  
    struct v4l2_subdev sd;
    struct media_pad pad;

    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *rd_reg;
    struct v4l2_ctrl *wr_reg;
    //struct v4l2_ctrl *refresh_reg;
    struct mutex mutex;
    bool streaming;
    bool identified;
    /* Other members as needed */
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

static int fpga_vir_cam_init_controls(struct fpga_vir_cam *fvc)
{
    struct v4l2_ctrl_handler *ctrl_hdlr;
    struct v4l2_ctrl_config reg_read_cfg = {0};
    struct v4l2_ctrl_config reg_write_cfg = {0};

    int ret;

    ctrl_hdlr = &fvc->ctrl_handler;
    ret = v4l2_ctrl_handler_init(ctrl_hdlr, 2);
    if (IS_ERR(ret))
        return ret;

    mutex_init(&fvc->mutex);
    ctrl_hdlr->lock = &fvc->mutex;

    reg_read_cfg.ops = &fpga_vir_cam_ctrl_ops;
    reg_read_cfg.id = V4L2_CID_PRIVATE_BASE + 0;
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
    reg_write_cfg.id = V4L2_CID_MY_REG_WRITE;
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
        v4l2_err(&fvc->v4l2_dev, "error while initializing control handler: %d\n", ret);
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

static int fpga_vir_cam_set_stream(struct v4l2_subdev *sd, int enable)
{
    struct fpga_vir_cam *fvc = to_fpga_vir_cam(sd);
    int ret;
    __u16 wrt_val;
    if(fvc->streaming == enable)
        return 0;
    mutex_lock(&fvc->mutex);
    if(enable){
        // Start streaming operations
        wrt_val = 1;
    } else {
        // Stop streaming operations
        wrt_val = 0;
    }
    ret = fpga_vir_cam_write_reg(fvc, FPGA_VIR_CAM_REG_START, wrt_val); 
    if(IS_ERR(ret)) {
        mutex_unlock(&fvc->mutex);
        return -EIO;
    }

    mutex_unlock(&fvc->mutex);
    return ret;
}

static const struct v4l2_subdev_video_ops fpga_video_ops = {
    .s_stream = fpga_vir_cam_set_stream,
};
static int fpga_vir_cam_free_controls(struct fpga_vir_cam *fvc)
{
    v4l2_ctrl_handler_free(&fvc->ctrl_handler);
    return 0;
}


static int fpga_vir_cam_probe(struct i2c_client *client)
{
    struct fpga_vir_cam *fvc;
    int ret;

    fvc = devm_kzalloc(&client->dev, sizeof(*fvc), GFP_KERNEL);
    if (!fvc)
        return -ENOMEM;

    v4l2_i2c_subdev_init(&fvc->sd, client, &fpga_video_ops);

    mutex_init(&fvc->mutex);
    fvc->streaming = false;
    ret = fpga_vir_cam_init_controls(fvc);
    if (IS_ERR(ret)) {
        dev_err(&client->dev, "failed to init controls: %d\n", ret);
        return ret;
    }
    /* Initialize controls here */

    ret = v4l2_async_register_subdev(&fvc->sd);
    if (ret)
        goto err_free_ctrls;

    return error;
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