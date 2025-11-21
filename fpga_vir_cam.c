#include <linux/acpi.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>


#define FPGA_VIR_CAM_REG_START   0x0001

struct fpga_vir_cam {
    struct v4l2_subdev sd;
    struct media_pad pad;

    struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl *rd_reg;
    struct v4l2_ctrl *wr_reg;
    struct mutex mutex;
    bool streaming;
    bool identified;
    /* Other members as needed */
};
#define to_fpga_vir_cam(_sd)	container_of(_sd, struct fpga_vir_cam, sd)
/* Read registers up to 4 at a time */
static int fpga_vir_cam_read_reg(struct fpga_vir_cam *fvc,
            unsigned short reg, unsigned int len, unsigned int *val)
{
    return 0;
}
static int fpga_vir_cam_write_reg(struct fpga_vir_cam *fvc,
            unsigned short reg, unsigned int len, unsigned int val)
{
    return 0;
}

static int fpga_vir_cam_set_stream(struct v4l2_subdev *sd, int enable)
{
    struct fpga_vir_cam *fvc = to_fpga_vir_cam(sd);
    int ret;
    unsigned int wrt_val;
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
    ret = fpga_vir_cam_write_reg(fvc, FPGA_VIR_CAM_REG_START , 1, wrt_val); // Example register
    if(ISERR(ret)) {
        mutex_unlock(&fvc->mutex);
        return -EIO;
    }

    mutex_unlock(&fvc->mutex);
    return ret;
}

static const struct v4l2_subdev_video_ops fpga_video_ops = {
    .s_stream = fpga_vir_cam_set_stream,
};

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

    /* Initialize controls here */

    ret = v4l2_async_register_subdev(&cam->sd);
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