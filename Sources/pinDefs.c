#include <pinDefs.h>

#include <stdbool.h>

#include <inc/hw_memmap.h>
#include <driverlib/gpio.h>
#include <driverlib/sysctl.h> //System Control

const PinDef LED1 = {SYSCTL_PERIPH_GPION, GPIO_PORTN_BASE, GPIO_PIN_1};
const PinDef LED2 = {SYSCTL_PERIPH_GPION, GPIO_PORTN_BASE, GPIO_PIN_0};
const PinDef LED3 = {SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, GPIO_PIN_4};
const PinDef LED4 = {SYSCTL_PERIPH_GPIOF, GPIO_PORTF_BASE, GPIO_PIN_0};
const PinDef bp_LED = {SYSCTL_PERIPH_GPIOE, GPIO_PORTE_BASE, GPIO_PIN_2};
const PinDef bp_S1 = {SYSCTL_PERIPH_GPIOE, GPIO_PORTE_BASE, GPIO_PIN_3};
const PinDef bp_S2 = {SYSCTL_PERIPH_GPIOD, GPIO_PORTD_BASE, GPIO_PIN_7};
