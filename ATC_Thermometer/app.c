#include <stdint.h>
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "vendor/common/blt_common.h"

#include "battery.h"
#include "ble.h"
#include "flash.h"
#include "lcd.h"
#include "sensor.h"


RAM uint32_t last_delay = 0xFFFF0000, last_adv_delay = 0xFFFF0000, last_battery_delay = 0xFFFF0000;
RAM bool last_smiley;
int16_t temp = 0;
uint16_t humi = 0;
RAM uint8_t adv_count = 0;
RAM uint8_t meas_count = 254;
RAM int16_t last_temp;
RAM uint16_t last_humi;
RAM uint8_t battery_level;
RAM uint16_t battery_mv;
RAM bool show_batt_or_humi;

//Settings
RAM bool temp_C_or_F;
RAM bool blinking_smiley = false;
RAM bool comfort_smiley = true;
RAM bool show_batt_enabled = true;
RAM bool advertising_type = false;//Custom or Mi Advertising (true)
RAM uint8_t advertising_interval = 6;//advise new values - multiply by 10 for value
RAM uint8_t measure_interval = 10;//time = loop interval * factor (def: about 7 * X)
RAM int8_t temp_offset;
RAM int8_t humi_offset;
RAM uint8_t temp_alarm_point = 5;//divide by ten for value
RAM uint8_t humi_alarm_point = 5;

RAM int16_t comfort_x[] = {2000, 2560, 2700, 2500, 2050, 1700, 1600, 1750};
RAM uint16_t comfort_y[] = {2000, 1980, 3200, 6000, 8200, 8600, 7700, 3800};


#define aht10_address 0b00100011

void bh1750_init(void) {
	/*
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, aht10_address << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00000001, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	*/

	i2c_set_id(aht10_address << 1);
	i2c_write_byte(0x00, 0, 0b00000001);
}

void bh1750_meassure(void) {
	/*
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, aht10_address << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0b00100000, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(120 / portTICK_PERIOD_MS);
	*/

	i2c_set_id(aht10_address << 1);
	i2c_write_byte(0x00, 0, 0b00100000);

	sleep_us(120 * 1000);
	//sleep_us(500);
}

uint16_t bh1750_getLux(void) {
    bh1750_meassure();

    uint8_t buffer[2];
	/*
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, aht10_address << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, buffer, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 100 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	*/

	i2c_set_id(aht10_address << 1);
	i2c_read_series(0x00, 0, (uint8_t*)buffer, 2);

    //uint16_t lux = (buffer[0] << 8 | buffer[1]) / 1.2;
	uint16_t lux = (buffer[0] << 8 | buffer[1]);

    //printf("[0]: %d, [1]: %d, Value: %d \n", buffer[0], buffer[1], lux);

    return lux;
}

_attribute_ram_code_ bool is_comfort(int16_t t, uint16_t h) {
    bool c = 0;
    uint8_t npol = sizeof(comfort_x);
    for (uint8_t i = 0, j = npol - 1; i < npol; j = i++) 
    {
      if ((
        (comfort_y[i] < comfort_y[j]) && (comfort_y[i] <= h) && (h <= comfort_y[j]) &&
        ((comfort_y[j] - comfort_y[i]) * (t - comfort_x[i]) > (comfort_x[j] - comfort_x[i]) * (h - comfort_y[i]))
      ) || (
        (comfort_y[i] > comfort_y[j]) && (comfort_y[j] <= h) && (h <= comfort_y[i]) &&
        ((comfort_y[j] - comfort_y[i]) * (t - comfort_x[i]) < (comfort_x[j] - comfort_x[i]) * (h - comfort_y[i]))
      ))
        c = !c;
    }
    return c;
}

void user_init_normal(void){//this will get executed one time after power up
	random_generator_init();  //must
	init_ble();	
	init_sensor();
	init_lcd();	
	init_flash();
	show_atc_mac();
	battery_mv = get_battery_mv();
	battery_level = get_battery_level(get_battery_mv());
}

_attribute_ram_code_ void user_init_deepRetn(void){//after sleep this will get executed
	blc_ll_initBasicMCU();
	rf_set_power_level_index (RF_POWER_P3p01dBm);
	blc_ll_recoverDeepRetention();
}


extern void app_uart_init(void);
extern void at_print(unsigned char * str);

void main_loop(){	
	if((clock_time()-last_delay) > 5000*CLOCK_SYS_CLOCK_1MS){//main loop delay
		

		bh1750_init();
		uint16_t lux = bh1750_getLux();
		
		//char gugus[50];
		//sprintf(gugus, "Measszrement: Lux: %d \r\n", lux);
		//at_print(gugus);

		if((clock_time()-last_battery_delay) > 5*60000*CLOCK_SYS_CLOCK_1MS){//Read battery delay
			battery_mv = get_battery_mv();
			battery_level = get_battery_level(get_battery_mv());
			last_battery_delay = clock_time();
		}

		if(meas_count >= measure_interval){
			read_sensor(&temp,&humi);		
			temp += temp_offset;
			humi += humi_offset;
			meas_count=0;
		
			if((temp-last_temp > temp_alarm_point)||(last_temp-temp > temp_alarm_point)||(humi-last_humi > humi_alarm_point)||(last_humi-humi > humi_alarm_point)){// instant advertise on to much sensor difference
				//set_adv_data(temp, humi, battery_level, battery_mv);
				set_adv_data_new(lux);
			}
			last_temp = temp;
			last_humi = humi;
		}	
		meas_count++;
		
		if(temp_C_or_F){
			show_temp_symbol(2);
			show_big_number(((((last_temp*10)/5)*9)+3200)/10,1);//convert C to F
		}else{
			show_temp_symbol(1);
			show_big_number(last_temp,1);
		}

		show_small_number(lux/100, 1);	
		show_battery_symbol(0);

/*
		if(!show_batt_enabled) show_batt_or_humi = true;
		
		if(show_batt_or_humi){//Change between Humidity displaying and battery level if show_batt_enabled=true
			show_small_number(last_humi,1);	
		    show_battery_symbol(0);   
		}else{
			show_small_number((battery_level==100)?99:battery_level,1);
			show_battery_symbol(1);
		}
*/

		show_batt_or_humi = !show_batt_or_humi;
		
		if(ble_get_connected()){//If connected notify Sensor data
			ble_send_temp(last_temp);
			ble_send_humi(last_humi);
			ble_send_battery(battery_level);
		}

		if((clock_time() - last_adv_delay) > (advertising_type?5000:10000)*CLOCK_SYS_CLOCK_1MS){//Advetise data delay
		    if(adv_count >= advertising_interval){
			//set_adv_data(last_temp, last_humi, battery_level, battery_mv);
			set_adv_data_new(lux);
			last_adv_delay = clock_time();
			adv_count=0;
		    }
		    adv_count++;
		}
		
		if(comfort_smiley) {
			if(is_comfort(last_temp * 10, last_humi * 100)){
				show_smiley(1);
			} else {
				show_smiley(2);
			}
		}

		if(blinking_smiley){//If Smiley should blink do it
		last_smiley=!last_smiley;
		show_smiley(last_smiley);
		}
		
		update_lcd();
		last_delay = clock_time();
	}
	blt_sdk_main_loop();
	blt_pm_proc();	
}
