#include <foc_interface.h>
#include <foc_utilities.h>

//==============================================================================================
#define def_FIELD_RANGE  32768
#define FIELD_CONTROLLER_LIMIT 2048 //2048

int field_control(int field_new, int field_e1, int field_e2, int q_value, int field_out_p_part, int field_out_i_part, int par_field_kp, int par_field_ki, int field_out1, int field_out2, int filter_sum[]){

    int control_output;

    field_e1       =   -field_new;   // 0 - field_new

    field_e2       = calc_hysteresis_and_limit(field_e1, 10, 80);


    if(q_value == 0)  // if q zero field_integrator must go to zero
    {
        if(field_out_i_part > 0) field_out_i_part -= field_out_i_part/8;
        if(field_out_i_part < 0) field_out_i_part -= field_out_i_part/8;
    }
    //===================================== field ===============================================
    field_out_p_part          =  field_e1 * par_field_kp;
    
    if(q_value == 0)  //  if q zero, reduce P part if it was too big
    {
        if(field_out_p_part > 0) field_out_p_part -= field_out_p_part/8;
        if(field_out_p_part < 0) field_out_p_part -= field_out_p_part/8;
    }

    field_out_i_part         +=   field_e2 * par_field_ki;
    field_out_i_part          =   check_limits(field_out_i_part, 512 * def_FIELD_RANGE);//512 * def_FIELD_RANGE


    field_out1 = (field_out_p_part + field_out_i_part);

    field_out2 =    low_pass_pt1_filter(filter_sum, ff_field_out, 32,  field_out1);//ToDo: check if it works properly

    control_output  = field_out2 / def_FIELD_RANGE;

    return control_output = calc_hysteresis_and_limit(control_output, 5, FIELD_CONTROLLER_LIMIT);//2048

}



//==============================================================================================
//  transformations functions
//==============================================================================================

{int,int} clarke_transformation(int curr_phase_b, int curr_phase_c)
{
int alpha,beta;

            alpha =   curr_phase_b;
            beta  =  (curr_phase_b + 2 * curr_phase_c);   // beta = (a1 + 2*a2)/1.732 0.57736 --> invers from 1.732

            beta *=  37838;
            beta /=  65536;

return{alpha,-beta};
}



{int, int} park_transformation(int sinus_tab, int cosinus_tab, int alpha, int beta )
{
int field_new, torque_new;

        field_new   = (((alpha * cosinus_tab )  /16384) + ((beta *  sinus_tab ) /16384));
        torque_new  = (((beta  * cosinus_tab )  /16384) - ((alpha * sinus_tab ) /16384));

return{field_new, torque_new};
}


{int, int} invers_park(int sinus_tab, int cosinus_tab, int field, int torque)
{
int park_alpha,park_beta;

        park_alpha = (field * cosinus_tab)/16384  -   (torque * sinus_tab)/16384;
        park_beta  = (field * sinus_tab)/16384    +   (torque * cosinus_tab)/16384;

return{park_alpha, park_beta};
}


int      adjust_angle_reference_pwm(int angle_inv_park, int angle_offset, int measure_tick, int speed_actual, int q_value, int filter_sum[])
{
int iTemp1;
int angle_new;
int angle_rpm_adjust;

  //  angle_new  = angle_inv_park  + 1024 + angle_offset;//why do we need 1024?
 //   angle_new  = angle_inv_park + angle_offset + 512;//why do we need 1024? For the hall sensor a correction of 1/4 of electrical rotation is required.

    if (q_value >= 0) angle_new  = angle_inv_park + angle_offset;
    else  angle_new  = angle_inv_park + angle_offset + 512;

    if(measure_tick)
    {
        iTemp1  = speed_actual * 300;// where does 300 come from?
        iTemp1  /= 4096;
        angle_rpm_adjust =  low_pass_pt1_filter(filter_sum, ffAngleRPM, 16, iTemp1);
    }

    angle_new += angle_rpm_adjust;
    angle_new &= 0x0FFF;

    return(angle_new);
}



