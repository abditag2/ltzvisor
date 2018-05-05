//
// Created by fardin on 5/3/18.
//

#include <stdint.h>

void double2string(double num) {

    if (num < 0){
        printk("-");
        num = -num;
    }

    unsigned int digit = 0;

    digit = (unsigned int) num/10000.0;
    printk("%d",digit);
    num = num - digit * 10000.0;

    digit = (unsigned int) num/1000.0;
    printk("%d",digit);
    num = num - digit * 1000.0;

    digit = (unsigned int) num/100.0;
    printk("%d",digit);
    num = num - digit * 100.0;

    digit = (unsigned int) num/10.0;
    printk("%d",digit);
    num = num - digit * 10.0;

    digit = (unsigned int) num;
    printk("%d",digit);
    num = num - digit;

    printk(".");

    num = num * 10.0;
    digit = (unsigned int) num;
    printk("%d",digit);
    num = num - digit;

    num = num * 10.0;
    digit = (unsigned int) num;
    printk("%d",digit);
    num = num - digit;

    num = num * 10.0;
    digit = (unsigned int) num;
    printk("%d",digit);
    num = num - digit;

    num = num * 10.0;
    digit = (unsigned int) num;
    printk("%d",digit);
    num = num - digit;

}

uint32_t safe_call_count = 0;


void safe_controller(){
    safe_call_count += 1;

    if (safe_call_count % 20 == 0){
        printk("called 20 times. %d\n", (uint32_t) safe_call_count/20);
    }
}