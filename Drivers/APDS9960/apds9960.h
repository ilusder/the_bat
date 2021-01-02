//******************************************************************************

//    _____ _               _____                        _     _
//   |_   _| |             |  __ \                      | |   (_)
//     | | | |_   _  __ _  | |  | | ___ _ __ _   _  __ _| |__  _ _ __
//     | | | | | | |/ _` | | |  | |/ _ \ '__| | | |/ _` | '_ \| | '_ \
//    _| |_| | |_| | (_| | | |__| |  __/ |  | |_| | (_| | |_) | | | | |
//   |_____|_|\__, |\__,_| |_____/ \___|_|   \__, |\__,_|_.__/|_|_| |_|
//             __/ |                          __/ |
//            |___/                          |___/

//******************************************************************************


#define APDS9960_REGMAP_NAME	"apds9960_regmap"
#define APDS9960_DRV_NAME	"apds9960"

#define APDS9960_REG_RAM_START	0x00
#define APDS9960_REG_RAM_END	0x7f

#define APDS9960_REG_ENABLE	0x80
#define APDS9960_REG_ATIME	0x81
#define APDS9960_REG_WTIME	0x83

#define APDS9960_REG_AILTL	0x84
#define APDS9960_REG_AILTH	0x85
#define APDS9960_REG_AIHTL	0x86
#define APDS9960_REG_AIHTH	0x87

#define APDS9960_REG_PILT	0x89
#define APDS9960_REG_PIHT	0x8b
#define APDS9960_REG_PERS	0x8c

#define APDS9960_REG_CONFIG_1	0x8d
#define APDS9960_REG_PPULSE	0x8e

#define APDS9960_REG_CONTROL	0x8f
#define APDS9960_REG_CONTROL_AGAIN_MASK		0x03
#define APDS9960_REG_CONTROL_PGAIN_MASK		0x0c
#define APDS9960_REG_CONTROL_AGAIN_MASK_SHIFT	0
#define APDS9960_REG_CONTROL_PGAIN_MASK_SHIFT	2

#define APDS9960_REG_CONFIG_2	0x90
#define APDS9960_REG_CONFIG_2_GGAIN_MASK	0x60
#define APDS9960_REG_CONFIG_2_GGAIN_MASK_SHIFT	5

#define APDS9960_REG_ID		0x92

#define APDS9960_REG_STATUS	0x93
#define APDS9960_REG_STATUS_PS_INT	BIT(5)
#define APDS9960_REG_STATUS_ALS_INT	BIT(4)
#define APDS9960_REG_STATUS_GINT	BIT(2)

#define APDS9960_REG_PDATA	0x9c
#define APDS9960_REG_POFFSET_UR	0x9d
#define APDS9960_REG_POFFSET_DL 0x9e
#define APDS9960_REG_CONFIG_3	0x9f

#define APDS9960_REG_GPENTH	0xa0
#define APDS9960_REG_GEXTH	0xa1

#define APDS9960_REG_GCONF_1	0xa2
#define APDS9960_REG_GCONF_1_GFIFO_THRES_MASK		0xc0
#define APDS9960_REG_GCONF_1_GFIFO_THRES_MASK_SHIFT	6

#define APDS9960_REG_GCONF_2	0xa3
#define APDS9960_REG_GOFFSET_U	0xa4
#define APDS9960_REG_GOFFSET_D	0xa5
#define APDS9960_REG_GPULSE	0xa6
#define APDS9960_REG_GOFFSET_L	0xa7
#define APDS9960_REG_GOFFSET_R	0xa9
#define APDS9960_REG_GCONF_3	0xaa

#define APDS9960_REG_GCONF_4	0xab
#define APDS9960_REG_GFLVL	0xae
#define APDS9960_REG_GSTATUS	0xaf

#define APDS9960_REG_IFORCE	0xe4
#define APDS9960_REG_PICLEAR	0xe5
#define APDS9960_REG_CICLEAR	0xe6
#define APDS9960_REG_AICLEAR	0xe7

#define APDS9960_DEFAULT_PERS	0x33
#define APDS9960_DEFAULT_GPENTH	0x50
#define APDS9960_DEFAULT_GEXTH	0x40

#define APDS9960_MAX_PXS_THRES_VAL	255
#define APDS9960_MAX_ALS_THRES_VAL	0xffff
#define APDS9960_MAX_INT_TIME_IN_US	1000000

enum apds9960_als_channel_idx {
	IDX_ALS_CLEAR, IDX_ALS_RED, IDX_ALS_GREEN, IDX_ALS_BLUE,
};

#define APDS9960_REG_ALS_BASE	0x94
#define APDS9960_REG_ALS_CHANNEL(_colour) \
	(APDS9960_REG_ALS_BASE + (IDX_ALS_##_colour * 2))

enum apds9960_gesture_channel_idx {
	IDX_DIR_UP, IDX_DIR_DOWN, IDX_DIR_LEFT, IDX_DIR_RIGHT,
};