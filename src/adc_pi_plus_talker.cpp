#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>


int file, file2;
char filename[11];// = "/dev/i2c-1";
__u8 addr1 = 0x68;         // The I2C address of the first ADC
__u8 addr2 = 0x69;	  // The I2C address of the second ADC
__u8 config1 = 0x9C;	// PGAx1, 18 bit, continuous conversion, channel 1
__u8 config2 = 0x9C;	// PGAx1, 18 bit, continuous conversion, channel 1
__u8 currentchannel1 = 1;  // channel variable for adc 1
__u8 currentchannel2 = 1;  // channel variable for adc2
__u8 bitrate = 18;
__u8 conversion_mode = 1;
float pga = (float)0.5;  // current pga setting
float lsb = (float)0.0000078125;  // default lsb value for 18 bit
bool signBit;
__u8 adcreading[4];



// internal method for setting the value of a single bit within a
//byte
__u8 updateByte(__u8 byte, __u8 bit, __u8 value){
	if (value == 0)
            return byte & ~(1 << bit);
        else if (value == 1)
            return byte | (1 << bit);
}

// internal method for reading the value of a single bit within a
//byte
__u8 checkBit(int byte, __u8 bit){
	__u8 bitval = ((byte & (1 << bit)) != 0);
        if (bitval == 1)
            return 1;
        else
            return 0;
}


// internal method for updating the config to the selected channel
void setChannel(__u8 channel){
	if (channel < 5){
            if (channel != currentchannel1){
                if (channel == 1) {
                    config1 = updateByte(config1, 5, 0);
                    config1 = updateByte(config1, 6, 0);
                    currentchannel1 = 1;
		}
                if (channel == 2){
                    config1 = updateByte(config1, 5, 1);
                    config1 = updateByte(config1, 6, 0);
                    currentchannel1 = 2;
		}
                if (channel == 3){
                    config1 = updateByte(config1, 5, 0);
                    config1 = updateByte(config1, 6, 1);
                    currentchannel1 = 3;
		}
                if (channel == 4){
                    config1 = updateByte(config1, 5, 1);
                    config1 = updateByte(config1, 6, 1);
                    currentchannel1 = 4;
		}
	    }
	}
        else {
            if (channel != currentchannel2){
                if (channel == 5){
                    config2 = updateByte(config2, 5, 0);
                    config2 = updateByte(config2, 6, 0);
                    currentchannel2 = 5;
		}
                if (channel == 6){
                    config2 = updateByte(config2, 5, 1);
                    config2 = updateByte(config2, 6, 0);
                    currentchannel2 = 6;
		}
                if (channel == 7){
                    config2 = updateByte(config2, 5, 0);
                    config2 = updateByte(config2, 6, 1);
                    currentchannel2 = 7;
		}
                if (channel == 8){
                    config2 = updateByte(config2, 5, 1);
                    config2 = updateByte(config2, 6, 1);
                    currentchannel2 = 8;
		}
	   }
	}
}

// reads the raw value from the selected adc channel - channels 1 to 8
float readRaw(__u8 channel){
	__u8 h = 0;
        __u8 l = 0;
	__u8 m = 0;
        __u8 s = 0;
	__u8 config;
	__u8 address;
	int fh;

        // get the config and i2c address for the selected channel
        setChannel(channel);
        if (channel < 5){            
            config = config1;
            address = addr1;
	    fh = file;
	}
        else{
            config = config2;
            address = addr2;
	    fh = file2;
	}
            
        // if the conversion mode is set to one-shot update the ready bit to 1
        if (conversion_mode == 0){
                config = updateByte(config, 7, 1);
		int res = i2c_smbus_write_byte_data(file, address, config);
		if(res < 0)
			perror("erro100");

                config = updateByte(config, 7, 0);
	}

        // keep reading the adc data until the conversion result is ready
	int a=1;
        while (a==1){
            
		//nanosleep((const struct timespec[]){{0, 5000000L}}, NULL); 
		i2c_smbus_read_i2c_block_data(fh,config,4,adcreading);

		//nanosleep((const struct timespec[]){{0, 5000000L}}, NULL); 


            if (bitrate == 18){
                h = adcreading[0];
                m = adcreading[1];
                l = adcreading[2];
                s = adcreading[3];
	   }
            else{
                h = adcreading[0];
                m = adcreading[1];
                s = adcreading[2];
	   }
            if (checkBit(s, 7) == 0)
                a=0;
	}

        signBit = 0;
        float t = 0.0;
        // extract the returned bytes and combine in the correct order
        if (bitrate == 18){
            t = ((h & 0b00000011) << 16) | (m << 8) | l;
            signBit = (bool)checkBit(t, 17);
            if (signBit)
                t = updateByte(t, 17, 0);
	}
        if (bitrate == 16){
            t = (h << 8) | m;
            signBit = (bool)checkBit(t, 15);
            if (signBit)
                t = updateByte(t, 15, 0);
	}

        if (bitrate == 14){
            t = ((h & 0b00111111) << 8) | m;
            signBit = (bool)checkBit(t, 13);
            if (signBit)
                t = updateByte(t, 13, 0);
	}

        if (bitrate == 12){
            t = ((h & 0b00001111) << 8) | m;
            signBit = (bool)checkBit(t, 11);
            if (signBit)
                t = updateByte(t, 11, 0);
	}

        return t;
}


// returns the voltage from the selected adc channel - channels 1 to
//8
float readVoltage(__u8 channel){
	float raw = readRaw(channel);
        if (signBit){
            return (float)0.0;  // returned a negative voltage so return 0
	}
        else {
            float voltage = (float)((raw * (lsb / pga)) * 2.471);
            return (float)voltage;	
	}
}

void setPGA(__u8 gain){
/*
        PGA gain selection
        1 = 1x
        2 = 2x
        4 = 4x
        8 = 8x
        */

        if (gain == 1){
            config1 = updateByte(config1, 0, 0);
            config1 = updateByte(config1, 1, 0);
            config2 = updateByte(config2, 0, 0);
            config2 = updateByte(config2, 1, 0);
            pga = 0.5;
	}
        if (gain == 2){
            config1 = updateByte(config1, 0, 1);
            config1 = updateByte(config1, 1, 0);
	    config2 = updateByte(config2, 0, 1);
            config2 = updateByte(config2, 1, 0);
            pga = 1;
	}
        if (gain == 4){
            config1 = updateByte(config1, 0, 0);
            config1 = updateByte(config1, 1, 1);
            config2 = updateByte(config2, 0, 0);
            config2 = updateByte(config2, 1, 1);
            pga = 2;
	}
        if (gain == 8){
            config1 = updateByte(config1, 0, 1);
            config1 = updateByte(config1, 1, 1);
            config2 = updateByte(config2, 0, 1);
            config2 = updateByte(config2, 1, 1);
            pga = 4;
	}

	int res = i2c_smbus_write_byte_data(file, addr1, config1);
	if(res < 0)
		perror("erro10");

	res = i2c_smbus_write_byte_data(file2, addr2, config2);
	if(res < 0)
		perror("erro20");

}

void setBitRate(__u8 rate){
/*
        sample rate and resolution
        12 = 12 bit (240SPS max)
        14 = 14 bit (60SPS max)
        16 = 16 bit (15SPS max)
        18 = 18 bit (3.75SPS max)
  */

        if (rate == 12){
            config1 = updateByte(config1, 2, 0);
            config1 = updateByte(config1, 3, 0);
            config2 = updateByte(config2, 2, 0);
            config2 = updateByte(config2, 3, 0);
            bitrate = 12;
            lsb = 0.0005;
	}
        if (rate == 14){
            config1 = updateByte(config1, 2, 1);
            config1 = updateByte(config1, 3, 0);
            config2 = updateByte(config2, 2, 1);
            config2 = updateByte(config2, 3, 0);
            bitrate = 14;
            lsb = 0.000125;
	}
        if (rate == 16){
            config1 = updateByte(config1, 2, 0);
            config1 = updateByte(config1, 3, 1);
            config2 = updateByte(config2, 2, 0);
            config2 = updateByte(config2, 3, 1);
            bitrate = 16;
            lsb = 0.00003125;
	}
        if (rate == 18){
            config1 = updateByte(config1, 2, 1);
            config1 = updateByte(config1, 3, 1);
            config2 = updateByte(config2, 2, 1);
            config2 = updateByte(config2, 3, 1);
            bitrate = 18;
            lsb = 0.0000078125;
	}

	int res = i2c_smbus_write_byte_data(file, addr1, config1);
	if(res < 0)
		perror("erro1");

	res = i2c_smbus_write_byte_data(file2, addr2, config2);
	if(res < 0)
		perror("erro2");

}

void setConversionMode(__u8 mode){
/*
        conversion mode for adc
        0 = One shot conversion mode
        1 = Continuous conversion mode
  */
        if (mode == 0){
            config1 = updateByte(config1, 4, 0);
            config2 = updateByte(config2, 4, 0);
            conversion_mode = 0;
	}
        if (mode == 1){
            config1 = updateByte(config1, 4, 1);
            config2 = updateByte(config2, 4, 1);
            conversion_mode = 1;
	}

	int res = i2c_smbus_write_byte_data(file, addr1, config1);
	if(res < 0)
		perror("erro1");

	res = i2c_smbus_write_byte_data(file2, addr2, config2);
	if(res < 0)
		perror("erro2");
}


void initADCPiPlusHat(__u8 address, __u8 address2, __u8 rate){
        addr1 = address;
        addr2 = address2;

	snprintf(filename, sizeof(filename), "/dev/i2c-%d", 1);	

	if ((file = open(filename, O_RDWR)) < 0) {
    		/* ERROR HANDLING: you can check errno to see what went wrong */
    		perror("Failed to open the i2c bus");
    		exit(1);
	}
	if ((file2 = open(filename, O_RDWR)) < 0) {
    		/* ERROR HANDLING: you can check errno to see what went wrong */
    		perror("Failed to open the i2c bus2");
    		exit(1);
	}

	if (ioctl(file,I2C_SLAVE,addr1) < 0) {
	    	printf("Failed to acquire bus access and/or talk to slave 1.\n");
    		// ERROR HANDLING; you can check errno to see what went wrong 
    		exit(1);
	}

	if (ioctl(file2,I2C_SLAVE,addr2) < 0) {
	    	printf("Failed to acquire bus access and/or talk to slave 2.\n");
    		// ERROR HANDLING; you can check errno to see what went wrong
    		exit(1);
	}


        setBitRate(rate);
}

void closeADCPiPlusHat(){
	close(file);
	close(file2);
}


int main(int argc, char **argv) {
	std_msgs::Float32MultiArray msg;

	initADCPiPlusHat(0x68, 0x69, 18);

	ros::init(argc, argv, "adc_pi_plus_talker");

	ros::NodeHandle n;

	ros::Publisher adc_pi_plus_pub = n.advertise<std_msgs::Float32MultiArray>("adc_pi_plus_pub", 1000);

	ros::Rate loop_rate(3);


	while (ros::ok())
	{	
		msg.data.clear();

		
		msg.data.push_back(readVoltage(1));
		msg.data.push_back(readVoltage(2));
		msg.data.push_back(readVoltage(3));
		msg.data.push_back(readVoltage(4));
		msg.data.push_back(readVoltage(5));
		msg.data.push_back(readVoltage(6));
		msg.data.push_back(readVoltage(7));
		msg.data.push_back(readVoltage(8));

		//ROS_INFO("%f", msg.data);

		adc_pi_plus_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}

	closeADCPiPlusHat();
	
	return 0;
}


