#pragma once

#include <cstdint>

namespace legged_waist_controller {

enum class RobotMotType
{
    // MOT_TYPE_NONE = 0,

    // // 基础运动
    // MOT_TYPE_LOCO = 1,

    // // 回home位置
    // MOT_TYPE_HOME = 51,

    // // 特殊运动
    // MOT_TYPE_STAND_UP = 101,
    // MOT_TYPE_SQUAT_DOWN = 102,

    // // 跳舞
    // MOT_TYPE_DANCE_1 = 201,
    // MOT_TYPE_DANCE_2 = 202,

    MOT_TYPE_NONE = 0,  //no request

    //basic operations
    MOT_TYPE_ENABLE = 901,
    MOT_TYPE_DISABLE = 902,
    MOT_TYPE_RESET = 903,
    MOT_TYPE_GO_HOME = 904,  //need to set target/home first
    MOT_TYPE_GO_LOCO_INITPOS = 905,
    MOT_TYPE_GO_MIMIC_INITPOS = 906,
    MOT_TYPE_PLAY_MIMIC = 907,
    MOT_TYPE_ABORT = 999,  //goto FAULT and disable robot

    //FSM_JUMPS
    MOT_TYPE_SWITCH_TO_FALUT = 1,
    MOT_TYPE_SWITCH_TO_START_FROM_LAYUP = 2,
    MOT_TYPE_SWITCH_TO_START_FROM_LAYDOWN = 3,
    MOT_TYPE_SWITCH_TO_IDLE = 7,
    MOT_TYPE_SWITCH_TO_LOCO = 8,
    MOT_TYPE_SWITCH_TO_MIMIC = 9,
    MOT_TYPE_SWITCH_TO_NOTARGET_HOLD = 10,
    MOT_TYPE_SWITCH_TO_NOTARGET = 11,
    MOT_TYPE_SWITCH_TO_REMOTE = 12,
    MOT_TYPE_SWITCH_TO_USER = 13,
    MOT_TYPE_SWITCH_TO_FSM_JAKAMINIMIMIC = 14,

    // NOTARGET_MOTIONS
    MOT_TYPE_NOD_HEAD = 101,
    MOT_TYPE_SHAKE_HEAD = 102,
    MOT_TYPE_WAVEHAND_LEFT = 103,
    MOT_TYPE_WAVEHAND_RIGHT = 104,
    MOT_TYPE_WAVEHANDS_LOW = 105,
    MOT_TYPE_WAVEHANDS_HIGH = 106,
    MOT_TYPE_SOLUTE_RIGHT = 107,
    MOT_TYPE_SHAKEHAND_RIGHT = 108,
    MOT_TYPE_HUG = 109,
    MOT_TYPE_YAOTOUHUANGNAO = 110,
    MOT_TYPE_CLAPHANDS = 111,
    MOT_TYPE_SHY = 112,
    MOT_TYPE_GUANWANG = 113,
    MOT_TYPE_CHEERS = 114,
    MOT_TYPE_HUAXIN = 117,
    MOT_TYPE_JIAORENGUOLAI = 118,
    MOT_TYPE_JIAYOU = 119,
    MOT_TYPE_JUJUE = 120,
    MOT_TYPE_JUSHOU = 121,
    MOT_TYPE_KEAI = 122,
    MOT_TYPE_NAOTOU = 123,
    MOT_TYPE_QIDAI = 124,
    MOT_TYPE_SIKAO = 125,
    MOT_TYPE_WULIAO = 126,

    //dance motions
    MOT_TYPE_DANCE1 = 501,
    MOT_TYPE_DANCE2 = 502,
    MOT_TYPE_DANCE3 = 503,
    MOT_TYPE_DANCE4 = 504,
    MOT_TYPE_DANCE5 = 505,
    MOT_TYPE_DANCE6 = 506,
    MOT_TYPE_DANCE7 = 507,
    MOT_TYPE_DANCE8 = 508,
    MOT_TYPE_DANCE9 = 509,
    MOT_TYPE_DANCE10 = 510,
};

struct MotCommand
{
    RobotMotType mot_type{RobotMotType::MOT_TYPE_NONE};
    /// 0 表示无有效指令；有效指令从 1 起单调递增
    uint64_t seq_id{0};
};

enum class MotCommandPolicy : uint8_t
{
    KeepLatest = 0,
    ConsumeLatest = 1,
    Queue = 2,
};

}  // namespace legged_waist_controller
