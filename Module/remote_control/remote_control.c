#include "remote_control.h"

#include "referee.h"

#define USE_VT_LINK (false)

#define PRESS_SEN 6

ATTR_PLACE_AT_NONCACHEABLE static remote_control_s remote_control;

void RemoteControlInit(void)
{
    memset(&remote_control, 0, sizeof(remote_control_s));

    remote_control.vt_link_data = getVtLinkRemoteControlData();

    remote_control.cc_data = getCustomerControllerData();
}

void RemoteControlUpdate(void)
{
    // 常规遥控控制器

#if USE_VT_LINK
    int temp_value = 0;
    remote_control.mouse_x = (temp_value = remote_control.dt7_dr16_data.mouse.x,
                              temp_value ? temp_value : (remote_control.vt_link_data->mouse_x));
    remote_control.mouse_y = (temp_value = remote_control.dt7_dr16_data.mouse.y,
                              temp_value ? temp_value : (remote_control.vt_link_data->mouse_y));
    remote_control.mouse_z = (temp_value = remote_control.dt7_dr16_data.mouse.z,
                              temp_value ? temp_value : (remote_control.vt_link_data->mouse_z));
    remote_control.rc_keys[RC_LEFT].is_pressed =
        (temp_value = remote_control.dt7_dr16_data.mouse.press_l,
         temp_value ? temp_value : ((uint8_t)remote_control.vt_link_data->left_button_down));
    remote_control.rc_keys[RC_RIGHT].is_pressed =
        (temp_value = remote_control.dt7_dr16_data.mouse.press_r,
         temp_value ? temp_value : ((uint8_t)remote_control.vt_link_data->right_button_down));
    for (uint8_t i = RC_W; i <= RC_B; i++)
    {
        remote_control.rc_keys[i].is_pressed =
            (temp_value = ((remote_control.dt7_dr16_data.key.v >> (i - RC_W)) & 1),
             temp_value ? temp_value : ((remote_control.vt_link_data->keyboard_value >> (i - RC_W)) & 1));
    }
#else
    remote_control.mouse_x = remote_control.dt7_dr16_data.mouse.x;
    remote_control.mouse_y = remote_control.dt7_dr16_data.mouse.y;
    remote_control.mouse_z = remote_control.dt7_dr16_data.mouse.z;
    remote_control.rc_keys[RC_LEFT].is_pressed = remote_control.dt7_dr16_data.mouse.press_l;
    remote_control.rc_keys[RC_RIGHT].is_pressed = remote_control.dt7_dr16_data.mouse.press_r;
    for (uint8_t i = RC_W; i <= RC_B; i++)
    {
        remote_control.rc_keys[i].is_pressed = ((remote_control.dt7_dr16_data.key.v >> (i - RC_W)) & 1);
    }
#endif

    for (uint8_t i = 0; i < RC_KEY_NUM; i++)
    {
        if (remote_control.rc_keys[i].is_pressed)
        {
            remote_control.rc_keys[i].press_timer =
                (remote_control.rc_keys[i].press_timer < 255) ? (remote_control.rc_keys[i].press_timer + 1) : 255;
            if (remote_control.rc_keys[i].press_timer == PRESS_SEN)
            {
                remote_control.rc_keys[i].falling_edge_detected = 1;
                remote_control.rc_keys[i].was_pressed = 1;
            }
        }
        else
        {
            remote_control.rc_keys[i].press_timer = 0;
        }

        if (remote_control.rc_keys[i].was_pressed == 1 && !remote_control.rc_keys[i].is_pressed)
        {
            remote_control.rc_keys[i].rising_edge_detected = 1;
            remote_control.rc_keys[i].was_pressed = 0;
        }
    }

    // 自定义控制器

    for (uint8_t i = 0; i < CC_KEY_NUM; i++)
    {
        remote_control.cc_keys[i].is_pressed =
            checkIfCustomerControllerKeyPressed(remote_control.cc_data->key, i) ? 1 : 0;
    }

    for (uint8_t i = 0; i < CC_KEY_NUM; i++)
    {
        if (remote_control.cc_keys[i].is_pressed)
        {
            remote_control.cc_keys[i].press_timer =
                (remote_control.cc_keys[i].press_timer < 255) ? (remote_control.cc_keys[i].press_timer + 1) : 255;
            if (remote_control.cc_keys[i].press_timer == PRESS_SEN)
            {
                remote_control.cc_keys[i].falling_edge_detected = 1;
                remote_control.cc_keys[i].was_pressed = 1;
            }
        }
        else
        {
            remote_control.cc_keys[i].press_timer = 0;
        }

        if (remote_control.cc_keys[i].was_pressed == 1 && !remote_control.cc_keys[i].is_pressed)
        {
            remote_control.cc_keys[i].rising_edge_detected = 1;
            remote_control.cc_keys[i].was_pressed = 0;
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
    return remote_control.rc_keys[key].is_pressed;
}

uint8_t checkIfRcKeyFallingEdgeDetected(remote_control_key_e key)
{
    if (remote_control.rc_keys[key].falling_edge_detected)
        return remote_control.rc_keys[key].falling_edge_detected = 0, 1;

    return 0;
}

uint8_t checkIfRcKeyRisingEdgeDetected(remote_control_key_e key)
{
    if (remote_control.cc_keys[key].rising_edge_detected)
        return remote_control.rc_keys[key].rising_edge_detected = 0, 1;

    return 0;
}

float getCcPose(uint8_t pose_index)
{
    return remote_control.cc_data->pose[pose_index];
}

uint8_t checkIsCcKeyPressed(customer_controller_key_index_e key)
{
    return remote_control.cc_keys[key].is_pressed;
}

uint8_t checkIfCcKeyFallingEdgeDetected(customer_controller_key_index_e key)
{
    if (remote_control.cc_keys[key].falling_edge_detected)
        return remote_control.cc_keys[key].falling_edge_detected = 0, 1;

    return 0;
}

uint8_t checkIfCcKeyRisingEdgeDetected(customer_controller_key_index_e key)
{
    if (remote_control.rc_keys[key].rising_edge_detected)
        return remote_control.rc_keys[key].rising_edge_detected = 0, 1;

    return 0;
}

remote_control_s *getRemoteControlPointer(void)
{
    return &remote_control;
}
