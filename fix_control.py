import sys

with open(r'msp_test\MSPM0G_CAR_K230\BSP\control\control.c', 'r', encoding='utf-8', errors='replace') as f:
    content = f.read()

old = '''\t\t\t/* === 编码器速差直接补偿（不等角度偏了再反应） === */
\t\t\tfloat speed_l = motor_data.speed_mm_s[0];
\t\t\tfloat speed_r = motor_data.speed_mm_s[1];
\t\t\tfloat speed_diff = speed_l - speed_r;     /* 正值=左快右慢 */
\t\t\tfloat trim = speed_diff * 2.0f;           /* 补偿量：速差×系数 */

\t\t\t/* === 合成PWM输出 === */
\t\t\tint base_pwm = 350;
\t\t\tint left_pwm  = base_pwm + (int)yaw_smooth - (int)trim;
\t\t\tint right_pwm = base_pwm - (int)yaw_smooth + (int)trim;'''

new = '''\t\t\t/* === 合成PWM：固定基准 + yaw差速 + 右轮偏置 === */
\t\t\tint base_pwm = 350;
\t\t\tint left_pwm  = base_pwm + (int)yaw_smooth;
\t\t\tint right_pwm = base_pwm - (int)yaw_smooth + RIGHT_BIAS;'''

if old in content:
    content = content.replace(old, new)
    # Add RIGHT_BIAS define
    old_comp = '#define RIGHT_COMP 1.00f\n'
    new_comp = '#define RIGHT_COMP 1.00f\n\n/* 右轮机械偏慢的固定PWM补偿 */\n#define RIGHT_BIAS  20\n'
    content = content.replace(old_comp, new_comp)

    with open(r'msp_test\MSPM0G_CAR_K230\BSP\control\control.c', 'w', encoding='utf-8', newline='') as f:
        f.write(content)
    print('OK')
else:
    print('OLD STRING NOT FOUND')
    # Debug: find what's actually there
    idx = content.find('编码器速差')
    if idx >= 0:
        print('Found at', idx)
        print(repr(content[idx:idx+300]))
    else:
        print('String not found at all')
