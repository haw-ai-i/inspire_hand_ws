# Inspire Hand DDS Topics

This document describes the primary Inspire hand DDS topics used in this workspace.

## Topic Overview

- Left hand control: `rt/inspire_hand/ctrl/l`
- Right hand control: `rt/inspire_hand/ctrl/r`
- Left hand state: `rt/inspire_hand/state/l`
- Right hand state: `rt/inspire_hand/state/r`
- Left hand touch: `rt/inspire_hand/touch/l`
- Right hand touch: `rt/inspire_hand/touch/r`

## Message Types (from `cyclonedds typeof`)

### Touch: `rt/inspire_hand/touch/l`

```idl
module inspire {
    @final
    struct inspire_hand_touch {
        sequence<short, 9>  fingerone_tip_touch;
        sequence<short, 96>  fingerone_top_touch;
        sequence<short, 80>  fingerone_palm_touch;
        sequence<short, 9>  fingertwo_tip_touch;
        sequence<short, 96>  fingertwo_top_touch;
        sequence<short, 80>  fingertwo_palm_touch;
        sequence<short, 9>  fingerthree_tip_touch;
        sequence<short, 96>  fingerthree_top_touch;
        sequence<short, 80>  fingerthree_palm_touch;
        sequence<short, 9>  fingerfour_tip_touch;
        sequence<short, 96>  fingerfour_top_touch;
        sequence<short, 80>  fingerfour_palm_touch;
        sequence<short, 9>  fingerfive_tip_touch;
        sequence<short, 96>  fingerfive_top_touch;
        sequence<short, 9>  fingerfive_middle_touch;
        sequence<short, 96>  fingerfive_palm_touch;
        sequence<short, 112>  palm_touch;
    };
};
```

### Control: `rt/inspire_hand/ctrl/l`

```idl
module inspire {
    @final
    struct inspire_hand_ctrl {
        sequence<short, 6>  pos_set;
        sequence<short, 6>  angle_set;
        sequence<short, 6>  force_set;
        sequence<short, 6>  speed_set;
        char mode;
    };
};
```

### State: `rt/inspire_hand/state/l`

```idl
module inspire {
    @final
    struct inspire_hand_state {
        sequence<short, 6>  pos_act;
        sequence<short, 6>  angle_act;
        sequence<short, 6>  force_act;
        sequence<short, 6>  current;
        sequence<octet, 6>  err;
        sequence<octet, 6>  status;
        sequence<octet, 6>  temperature;
    };
};
```

## Field Meaning (practical)

- `pos_set` / `pos_act`: commanded/measured position per joint (6 channels).
- `angle_set` / `angle_act`: commanded/measured joint angle (6 channels).
- `force_set` / `force_act`: commanded/measured force (6 channels).
- `speed_set`: commanded speed per joint (6 channels).
- `current`: motor current per joint (6 channels).
- `err`: per-joint error code byte.
- `status`: per-joint status byte.
- `temperature`: per-joint temperature byte.
- `mode`: control mode bitfield for command interpretation.

## Useful Commands

```bash
cyclonedds typeof rt/inspire_hand/touch/l
cyclonedds typeof rt/inspire_hand/ctrl/l
cyclonedds typeof rt/inspire_hand/state/l

cyclonedds subscribe rt/inspire_hand/state/l
cyclonedds subscribe rt/inspire_hand/touch/l
```
