//fpga_vir_cam.h

#ifndef _FPGA_VIR_CAM_H__
#define _FPGA_VIR_CAM_H__

/*v4l2 ctrl id*/ 
#define FPGA_VIR_CAM_V4L2_REG_READ  0x08000000//(V4L2_CID_PRIVATE_BASE + 0)
#define FPGA_VIR_CAM_V4L2_REG_WRITE 0x08000001//(V4L2_CID_PRIVATE_BASE + 1)

struct fpga_vir_cam_reg {
    unsigned short reg_value;
    unsigned int reg_addr;
};


#endif