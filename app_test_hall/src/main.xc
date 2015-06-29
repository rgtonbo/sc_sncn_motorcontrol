/* PLEASE REPLACE "CORE_BOARD_REQUIRED" AND "IMF_BOARD_REQUIRED" WIT A APPROPRIATE BOARD SUPPORT FILE FROM module_board-support */
#include <CORE_BOARD_REQUIRED>
#include <IFM_BOARD_REQUIRED>

/**
 * @file test_hall.xc
 * @brief Test illustrates usage of hall sensor to get position and velocity information
 * @author Synapticon GmbH <support@synapticon.com>
 */

#include <print.h>
#include <hall_client.h>
#include <hall_server.h>
#include <xscope_wrapper.h>

//#define ENABLE_xscope

/* Test Hall Sensor Client */
void hall_test(chanend c_hall)
{
    int position = 0;
    int velocity = 0;
    int direction;

    while(1)
    {
        /* get position from Hall Sensor */
        {position, direction} = get_hall_position_absolute(c_hall);

        /* get velocity from Hall Sensor */
        velocity = get_hall_velocity(c_hall);

#ifdef ENABLE_xscope
        xscope_core_int(0, position);
        xscope_core_int(1, velocity);
#else
        printstr("Position: ");
        printint(position);
        printstr(" ");
        printstr("Velocity: ");
        printintln(velocity);
#endif
    }
}

int main(void)
{
    chan c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6;  // hall channels

    par
    {
        on tile[COM_TILE]:
        {
            /* Test Hall Sensor Client */
            par
            {
                hall_test(c_hall_p1);
            }
        }

        /************************************************************
         * IFM_TILE
         ************************************************************/
        on tile[IFM_TILE]:
        {
            /* Hall Server */
            {
                hall_par hall_params;
                run_hall(c_hall_p1, c_hall_p2, c_hall_p3, c_hall_p4, c_hall_p5, c_hall_p6,
                         p_ifm_hall, hall_params); // channel priority 1,2..6
            }

        }

    }

    return 0;
}