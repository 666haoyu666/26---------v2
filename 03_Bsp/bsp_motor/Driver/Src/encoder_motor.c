/**
 * @file    encoder_motor.c
 * @brief   编码器实现
 * @author  bojing
 * @note    本层只负责编码器的计数处理，
 *          GPIO 中断负责采集编码器读数
 */

#include "encoder_motor.h"

#define ENCODER_SPEED_LPF_ALPHA_DEFAULT 0.25f

/**
 * @brief 实例化编码器对象
 *
 * @param encoder          编码器对象
 * @param direction_sign   原始计数方向修正
 * @param counts_per_rev   输出轴旋转一圈的有效计数
 * @param sample_period_s  固定更新周期，单位：秒
 *
 * @retval ENCODER_OK
 * @retval ENCODER_ERRORPARAMETER
 */
encoder_status_t encoder_inst(
    encoder_t *const encoder,
    encoder_sign_t direction_sign,
    uint32_t counts_per_rev,
    float sample_period_s)
{
    encoder_status_t ret = ENCODER_OK;

    if (encoder == NULL)
    {
#ifdef DEBUG
        DEBUG_OUT("ENCODER_ERRORPARAMETER\r\n");
#endif // DEBUG          
        return ENCODER_ERRORPARAMETER;
    }

    if(direction_sign != ENCODER_SIGN_NORMAL && direction_sign != ENCODER_SIGN_REVERSED)
    {
#ifdef DEBUG
        DEBUG_OUT("ENCODER_ERRORPARAMETER\r\n");
#endif // DEBUG
        return ENCODER_ERRORPARAMETER;
    }

    if (counts_per_rev == 0)
    {
#ifdef DEBUG
        DEBUG_OUT("ENCODER_ERRORPARAMETER\r\n");
#endif // DEBUG
        return ENCODER_ERRORPARAMETER;
    }

    if(sample_period_s <= 0.0f)
    {
#ifdef DEBUG
        DEBUG_OUT("ENCODER_ERRORPARAMETER\r\n");
#endif // DEBUG
        return ENCODER_ERRORPARAMETER;
    }   

    encoder->direction_sign = direction_sign;
    encoder->counts_per_rev = counts_per_rev;
    encoder->sample_period_s = sample_period_s;

    encoder->pending_count = 0;
    encoder->delta_count = 0;
    encoder->position_ticks = 0;
		
    encoder->raw_speed_rps = 0.0f;
		encoder->speed_rps = 0.0f;
		encoder->filter_is_ready = 0U;
		encoder->filter_alpha =
				ENCODER_SPEED_LPF_ALPHA_DEFAULT;
		
    encoder->is_inited = 1;

    return ret;
}

/**
 * @brief 在 GPIO 中断中增加一次原始计数
 *
 * @param encoder 编码器对象
 * @param step    本次计数变化，只允许为 +1 或 -1
 *
 * @note
 * 本函数由 GPIO 中断服务函数调用。
 * step 由硬件中断层根据 A/B 相电平判断。
 */
void encoder_count_isr(
    encoder_t *const encoder,
    int8_t step)
{
    if (encoder == NULL || !encoder->is_inited)
    {
        return;
    }

    if ((step != 1) && (step != -1))
    {
        return;
    }

    encoder->pending_count += (int32_t)step;
}

/**
 * @brief 周期更新编码器速度和位置
 *
 * @param encoder 编码器对象
 *
 * @note
 * 调用本函数时，需要保证 pending_count 的读取和清零
 * 不会被 GPIO 中断打断。
 *
 * @retval ENCODER_OK
 * @retval ENCODER_ERRORPARAMETER
 */
encoder_status_t encoder_update(
    encoder_t *const encoder)
{
    encoder_status_t ret = ENCODER_OK;

    if (encoder == NULL)
    {
#ifdef DEBUG
        DEBUG_OUT("ENCODER_ERRORPARAMETER\r\n");
#endif // DEBUG         
        return ENCODER_ERRORPARAMETER;
    }

    if (!encoder->is_inited)
    {
#ifdef DEBUG
        DEBUG_OUT("ENCODER_ERROR\r\n");
#endif // DEBUG
        return ENCODER_ERROR;
    }

    /*读取并清零当前周期的计数*/
    int32_t now_count = encoder->pending_count;
    encoder->pending_count = 0;

    /*方向修正*/
    encoder->delta_count=now_count*encoder->direction_sign;

    /*更新位置*/
    encoder->position_ticks += encoder->delta_count;

    /*更新速度*/
    encoder->raw_speed_rps = (float)encoder->delta_count / 
													   (float)encoder->counts_per_rev / 
														  encoder->sample_period_s;

		if (encoder->filter_is_ready == 0U)
		{
				encoder->speed_rps =
						encoder->raw_speed_rps;

				encoder->filter_is_ready = 1U;
		}
		else
		{
				encoder->speed_rps =
						encoder->speed_rps +
						encoder->filter_alpha *
						(encoder->raw_speed_rps -
						 encoder->speed_rps);
		}
	
    return ret;
}

/**
 * @brief 清零编码器运行状态
 *
 * @param encoder 编码器对象
 *
 * @retval ENCODER_OK
 * @retval ENCODER_ERRORPARAMETER
 */
encoder_status_t encoder_reset(
    encoder_t *const encoder)
{
    encoder_status_t ret = ENCODER_OK;

    if (encoder == NULL) 
    {
#ifdef DEBUG
        DEBUG_OUT("ENCODER_ERRORPARAMETER\r\n");
#endif // DEBUG         
        return ENCODER_ERRORPARAMETER;
    }

    if (!encoder->is_inited)
    {
#ifdef DEBUG
        DEBUG_OUT("ENCODER_ERROR\r\n");
#endif // DEBUG
        return ENCODER_ERROR;
    }

    encoder->pending_count = 0;
    encoder->delta_count = 0;
    encoder->position_ticks = 0;
		
		encoder->raw_speed_rps = 0.0f;
		encoder->speed_rps = 0.0f;
		encoder->filter_is_ready = 0U;

    return ret;
}
