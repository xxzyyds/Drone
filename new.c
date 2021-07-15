#include "new.h"

int t;
void up()
{
	t=3000;
	while(t--)
	{
		set_alt_pos(50);
		kernel_polling();
	}
	t=2000;while(t--)kernel_polling();
}
void down()
{
	t=3000;
	while(t--)
	{
		set_alt_pos(-50);
		kernel_polling();
	}
	t=2000;while(t--)kernel_polling();
}
void forward()
{
	shell_control.exp_att.roll = 0;
	t=300;
	while(t--)
	{
		shell_control.exp_att.pitch = 30f;
		kernel_polling();
	}
	t=1000;while(t--) kernel_polling();
	t=300;
	while(t--)
	{
		shell_control.exp_att.pitch = -20f;
		kernel_polling();
	}
}
void back()
{
	shell_control.exp_att.roll = 0;
	t=300;
	while(t--)
	{
		shell_control.exp_att.pitch = -30f;
		kernel_polling();
	}
	t=1000;while(t--) kernel_polling();
	t=300;
	while(t--)
	{
		shell_control.exp_att.pitch = 20f;
		kernel_polling();
	}
}
void left()
{
	shell_control.exp_att.pitch = 0;
	t=300;
	while(t--)
	{
		shell_control.exp_att.roll = 30f;
		kernel_polling();
	}
	t=1000;while(t--) kernel_polling();
	t=300;
	while(t--)
	{
		shell_control.exp_att.roll = -20f;
		kernel_polling();
	}
}
void right()
{
	shell_control.exp_att.pitch = 0;
	t=300;
	while(t--)
	{
		shell_control.exp_att.roll = -30f;
		kernel_polling();
	}
	t=1000;while(t--) kernel_polling();
	t=300;
	while(t--)
	{
		shell_control.exp_att.roll = 20f;
		kernel_polling();
	}
}
void unlock()
{
	SystemInfo.FMUflgPtr->Mode = Altitude_Hold;
	fmu_disarmed();
}
void lock()
{
	fmu_armed();
	update_shell_wdog();
    update_fmu_status();
}
