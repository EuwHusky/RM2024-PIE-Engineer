#include "remote_control.h"

#include "referee.h"

#define PRESS_SEN 5

remote_control_s remote_control;

void RemoteControlInit(void)
{
    memset(&remote_control, 0, sizeof(remote_control_s));

    remote_control.vt_link_data = getVtLinkRemoteControlData();
}

void RemoteControlUpdate(void)
{
    int temp_value = 0;

    // remote_control.mouse_x = (temp_value = remote_control.dt7_dr16_data.mouse.x,
    //                           temp_value ? temp_value : (remote_control.vt_link_data->mouse_x));
    // remote_control.mouse_y = (temp_value = remote_control.dt7_dr16_data.mouse.y,
    //                           temp_value ? temp_value : (remote_control.vt_link_data->mouse_y));
    // remote_control.mouse_z = (temp_value = remote_control.dt7_dr16_data.mouse.z,
    //                           temp_value ? temp_value : (remote_control.vt_link_data->mouse_z));

    remote_control.mouse_x = remote_control.dt7_dr16_data.mouse.x;
    remote_control.mouse_y = remote_control.dt7_dr16_data.mouse.y;
    remote_control.mouse_z = remote_control.dt7_dr16_data.mouse.z;

    remote_control.keys[RC_LEFT].is_pressed =
        (temp_value = remote_control.dt7_dr16_data.mouse.press_l,
         temp_value ? temp_value : ((uint8_t)remote_control.vt_link_data->left_button_down));
    remote_control.keys[RC_RIGHT].is_pressed =
        (temp_value = remote_control.dt7_dr16_data.mouse.press_r,
         temp_value ? temp_value : ((uint8_t)remote_control.vt_link_data->right_button_down));

    for (uint8_t i = RC_W; i <= RC_B; i++)
    {
        // remote_control.keys[i].is_pressed =
        //     (temp_value = ((remote_control.dt7_dr16_data.key.v >> (i - RC_W)) & 1),
        //      temp_value ? temp_value : ((remote_control.vt_link_data->keyboard_value >> (i - RC_W)) & 1));
        remote_control.keys[i].is_pressed = (remote_control.dt7_dr16_data.key.v >> (i - RC_W)) & (1);
    }

    for (uint8_t i = 0; i < RC_KEY_NUM; i++)
    {
        if (remote_control.keys[i].is_pressed)
        {
            remote_control.keys[i].press_timer =
                (remote_control.keys[i].press_timer < 255) ? (remote_control.keys[i].press_timer + 1) : 255;
            if (remote_control.keys[i].press_timer == PRESS_SEN)
            {
                remote_control.keys[i].falling_edge_detected = 1;
                remote_control.keys[i].was_pressed = 1;
            }
        }
        else
        {
            remote_control.keys[i].press_timer = 0;
        }

        if (remote_control.keys[i].was_pressed == 1 && !remote_control.keys[i].is_pressed)
        {
            remote_control.keys[i].rising_edge_detected = 1;
            remote_control.keys[i].was_pressed = 0;
        }
    }
}

int16_t getRcMouseX(void)
{
    return remote_control.mouse_x;
}

int16_t getRcMouseY(void)
{
    return remote_control.mouse_y;
}

int16_t getRcMouseZ(void)
{
    return remote_control.mouse_z;
}

uint8_t checkIsRcKeyPressed(remote_control_key_e key)
{
    return remote_control.keys[key].is_pressed;
}

uint8_t checkIfRcKeyFallingEdgeDetected(remote_control_key_e key)
{
    if (remote_control.keys[key].falling_edge_detected)
        return remote_control.keys[key].falling_edge_detected = 0, 1;

    return 0;
}

uint8_t checkIfRcKeyRisingEdgeDetected(remote_control_key_e key)
{
    if (remote_control.keys[key].rising_edge_detected)
        return remote_control.keys[key].rising_edge_detected = 0, 1;

    return 0;
}

remote_control_s *getRemoteControlPointer(void)
{
    return &remote_control;
}
