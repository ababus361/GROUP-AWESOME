/**
  ******************************************************************************
  * @file    main.c
  * @author  Niraj Menon
  * @date    Sep 23, 2024
  * @brief   ECE 362 I2C Lab Student template
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include <stdio.h>

void internal_clock();
void enable_ports();
void init_i2c();
void i2c_start(uint32_t targadr, uint8_t size, uint8_t dir);
void i2c_stop();
void i2c_waitidle();
void i2c_clearnack();
int i2c_checknack();

//===========================================================================
// Configure SDA and SCL.
//===========================================================================
void enable_ports(void) {
    
}

//===========================================================================
// Configure I2C1.
//===========================================================================
void init_i2c(void) {
    
}

//===========================================================================
// Send a START bit.
//===========================================================================
void i2c_start(uint32_t targadr, uint8_t size, uint8_t dir) {
  
}

//===========================================================================
// Send a STOP bit.
//===========================================================================
void i2c_stop(void) {

}

//===========================================================================
// Wait until the I2C bus is not busy. (One-liner!)
//===========================================================================
void i2c_waitidle(void) {
  
}

//===========================================================================
// Send each char in data[size] to the I2C bus at targadr.
//===========================================================================
int8_t i2c_senddata(uint8_t targadr, uint8_t data[], uint8_t size) {
    
}

//===========================================================================
// Receive size chars from the I2C bus at targadr and store in data[size].
//===========================================================================
int i2c_recvdata(uint8_t targadr, void *data, uint8_t size) {
        
}

//===========================================================================
// Clear the NACK bit. (One-liner!)
//===========================================================================
void i2c_clearnack(void) {
    
}

//===========================================================================
// Check the NACK bit. (One-liner!)
//===========================================================================
int i2c_checknack(void) {
    
}

//===========================================================================
// EEPROM functions
// We'll give these so you don't have to figure out how to write to the EEPROM.
// These can differ by device.

#define EEPROM_ADDR 0x57

void eeprom_write(uint16_t loc, const char* data, uint8_t len) {
    uint8_t bytes[34];
    bytes[0] = loc>>8;
    bytes[1] = loc&0xFF;
    for(int i = 0; i<len; i++){
        bytes[i+2] = data[i];
    }
    i2c_senddata(EEPROM_ADDR, bytes, len+2);
}

void eeprom_read(uint16_t loc, char data[], uint8_t len) {
    // ... your code here
    uint8_t bytes[2];
    bytes[0] = loc>>8;
    bytes[1] = loc&0xFF;
    i2c_senddata(EEPROM_ADDR, bytes, 2);
    i2c_recvdata(EEPROM_ADDR, data, len);
}

//===========================================================================
// Copy in code from Lab 7 - USART
//===========================================================================

#include "fifo.h"
#include "tty.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;

//===========================================================================
// init_usart5
//===========================================================================


//===========================================================================
// enable_tty_interrupt
//===========================================================================


//===========================================================================
// interrupt_getchar
//===========================================================================


//===========================================================================
// __io_putchar
//===========================================================================


//===========================================================================
// __io_getchar
//===========================================================================


//===========================================================================
// IRQHandler for USART5
//===========================================================================


//===========================================================================
// Command functions for the shell
// These are the functions that are called when a command is entered, in 
// order to parse the command being entered, split into argc/argv calling 
// convention, and then execute the command by calling the respective 
// function.  We implement "write" and "read" for you to easily test your 
// EEPROM functions from a terminal.
//===========================================================================

void write(int argc, char* argv[]) {
    if (argc < 3) {
        printf("Usage: write <addr> <data>\n");
        printf("Ensure the address is a hexadecimal number.  No need to include 0x.\n");
        return;
    }
    uint32_t addr = atoi(argv[1]); 
    // concatenate all args from argv[2], until empty string is found, to a string
    char data[32] = "";
    int i = 0;
    int j = 2;
    while (strcmp(argv[j], "") != 0 && i < 32) {
        for (char c = argv[j][0]; c != '\0'; c = *(++argv[j])) {
            data[i++] = c;
        }
        if (strcmp(argv[j+1], "") != 0)
            data[i++] = ' ';
        j++;
    }
    // ensure addr is a multiple of 32
    if ((addr % 10) != 0) {
        printf("Address 0x%ld is not evenly divisible by 32.  Your address must be a hexadecimal value.\n", addr);
        return;
    }
    int msglen = strlen(data);
    if (msglen > 32) {
        printf("Data is too long. Max length is 32.\n");
        return;
    }
    printf("Writing to address 0x%ld: %s\n", addr, data);
    eeprom_write(addr, data, msglen);
}

void read(int argc, char* argv[]) {
    if (argc != 2) {
        printf("Usage: read <addr>\n");
        printf("Ensure the address is a hexadecimal number.  No need to include 0x.\n");
        return;
    }
    uint32_t addr = atoi(argv[1]); 
    char data[32];
    // ensure addr is a multiple of 32
    if ((addr % 10) != 0) {
        printf("Address 0x%ld is not evenly divisible by 32.  Your address must be a hexadecimal value.\n", addr);
        return;
    }
    eeprom_read(addr, data, 32);
    printf("String at address 0x%ld: %s\n", addr, data);
}

struct commands_t {
    const char *cmd;
    void      (*fn)(int argc, char *argv[]);
};

struct commands_t cmds[] = {
    { "write", write },
    { "read", read }
};

void exec(int argc, char *argv[])
{
    for(int i=0; i<sizeof cmds/sizeof cmds[0]; i++)
        if (strcmp(cmds[i].cmd, argv[0]) == 0) {
            cmds[i].fn(argc, argv);
            return;
        }
    printf("%s: No such command.\n", argv[0]);
}

void parse_command(char *c)
{
    char *argv[20];
    int argc=0;
    int skipspace=1;
    for(; *c; c++) {
        if (skipspace) {
            if (*c != ' ' && *c != '\t') {
                argv[argc++] = c;
                skipspace = 0;
            }
        } else {
            if (*c == ' ' || *c == '\t') {
                *c = '\0';
                skipspace=1;
            }
        }
    }
    if (argc > 0) {
        argv[argc] = "";
        exec(argc, argv);
    }
}

//===========================================================================
// main()
//===========================================================================

int main() {
    internal_clock();
    
    // I2C specific
    enable_ports();
    init_i2c();

    // If you don't want to deal with the command shell, you can 
    // comment out all code below and call 
    // eeprom_read/eeprom_write directly.
    init_usart5();
    enable_tty_interrupt();
    // These turn off buffering.
    setbuf(stdin,0); 
    setbuf(stdout,0);
    setbuf(stderr,0);

    printf("I2C Command Shell\n");
    printf("This is a simple shell that allows you to write to or read from the I2C EEPROM at %d.\n", EEPROM_ADDR);
    for(;;) {
        printf("\n> ");
        char line[100];
        fgets(line, 99, stdin);
        line[99] = '\0';
        int len = strlen(line);
        if (line[len-1] == '\n')
            line[len-1] = '\0';
        parse_command(line);
    }
}