#include "m328Pdef.inc"

.equ freq = 16000000 ; Hz

.equ cycles_per_us = freq / 1000000 ; 1 us = 10^-6 s

; I/O pins
#define TRIGGER_PIN 0
#define IR_PIN 1
#define SCOPE_PIN 2
#define DISPLAY_RESET_PIN 4
#define SUCCESS_LED 6
#define ERROR_LED 7

; Handy macros
#define SCOPE_PIN_ON  sbi PORTD, SCOPE_PIN
#define SCOPE_PIN_OFF cbi PORTD, SCOPE_PIN

#define SUCCESS_LED_ON  sbi PORTD, SUCCESS_LED
#define SUCCESS_LED_OFF cbi PORTD, SUCCESS_LED

#define ERROR_LED_ON  sbi PORTD, ERROR_LED
#define ERROR_LED_OFF cbi PORTD, ERROR_LED

.equ RAM_END = 32768

jmp init  ; RESET
jmp onirq ; INT0 - ext IRQ 0
jmp onirq ; INT1 - ext IRQ 1
jmp onirq ; PCINT0 - pin change IRQ 
jmp onirq ; PCINT1 - pin change IRQ
jmp onirq ; PCINT2 - pin change IRQ
jmp onirq ; WDT - watchdog IRQ
jmp onirq ; TIMER2_COMPA - timer/counter 2 compare match A
jmp onirq ; TIMER2_COMPB - timer/counter 2 compare match B
jmp onirq ; TIMER2_OVF - timer/counter 2 overflow
jmp onirq ; TIMER1_CAPT - timer/counter 1 capture event
jmp onirq ; TIMER1_COMPA 
jmp onirq ; TIMER1_COMPB
jmp onirq ; TIMER1_OVF
jmp onirq ; TIMER0_COMPA
jmp onirq ; TIMER0_COMPB
jmp onirq ; TIMER0_OVF
jmp onirq ; STC - serial transfer complete (SPI)
jmp onirq ; USUART Rx complete
jmp onirq ; USUART Data register empty
jmp onirq ; USUART Tx complete
jmp onirq ; ADC conversion complete
jmp onirq ; EEPROM ready
jmp onirq ; Analog comparator
jmp onirq ; 2-wire interface I2C
jmp onirq ; Store program memory ready

; ========================
; HW init
; ========================
init:
; clear status register
	eor r1,r1
	out 0x3f,r1
; initialize stack pointer
	ldi r28,LOW(RAM_END)
	ldi r29,HIGH(RAM_END)
	out 0x3d,r28	; SPL 
	out 0x3e,r29	; SPH
  
; call main program
.again	rcall main2
.die  
          rjmp die
  
onirq:
          ERROR_LED_ON
	ldi r16,0xff
          rcall msleep
	ldi r16,0xff
          rcall msleep
	ERROR_LED_OFF
	ldi r16,0xff
	rcall msleep
	ldi r16,0xff
          rcall msleep
          rjmp onirq

; ==========================
; main program starts here
; ==========================
main2:  
  
    ldi      r17, 0xff
    out      DDRD, r17  
    out      DDRB, r17    
    
    ; Den Pin OC1A zu guter letzt noch auf Ausgang schalten

    ;
    ; Timer 1 einstellen
    ;
    ; Modus 14:
    ;    Fast PWM, Top von ICR1
    ;
    ;     WGM13    WGM12   WGM11    WGM10
    ;      1        1       1        0
    ;
    ;    Timer Vorteiler: 256
    ;     CS12     CS11    CS10
    ;      1        0       0
    ;
    ; Steuerung des Ausgangsport: Set at BOTTOM, Clear at match
    ;     COM1A1   COM1A0
    ;      1        0
    ;
    ldi      r17, (1<<COM1A0) | 1<<COM1B0 | 1<<WGM11
    sts      TCCR1A, r17

    ldi      r17, (1<<WGM13) | 1<<WGM12 | 4
    sts      TCCR1B, r17  

    ;
    ; den Endwert (TOP) für den Zähler setzen
    ; der Zähler zählt bis zu diesem Wert
    ;
    ldi      r17, 0xff
    ldi      r16, 0xff  
    sts      ICR1H, r17
    sts      ICR1L, r16

    ;
    ; der Compare Wert
    ; Wenn der Zähler diesen Wert erreicht, wird mit
    ; obiger Konfiguration der OC1A Ausgang abgeschaltet
    ; Sobald der Zähler wieder bei 0 startet, wird der
    ; Ausgang wieder auf 1 gesetzt
    ;
    ldi      r17, 0x3F ; hi
    ldi      r16, 0x3F ; lo
    sts      OCR1AH, r17
    sts      OCR1AL, r16
    
.forever
    rjmp forever  

; =======
; Sleep up to 255 millseconds
; INPUT: r16 - time to sleep in ms
; ======= 
msleep:
	push r16
	push r25
	push r24
.loop
	rcall sleep_one_ms
	dec r16
	brne loop
	pop r24
	pop r25
	pop r16
	ret
; =====
; sleep one millisecond
; SCRATCHED: r25:r24
.equ loop_count = 16000/3-3 ; 16000 cycles = 1 ms , - 3 loop iterations (=9 cycles) for instructions before/after loop
sleep_one_ms:
	ldi r24, LOW(loop_count)
	ldi r25, HIGH(loop_count)
	nop
	nop
.loop1
	sbiw r25:r24,1 ; 2 cycles
	brne loop1 ; 1 if condition is false, otherwise 2 
	ret ; 4 cles

; =========
; sleep for up to 255 micro seconds
;
; >>>> Must NEVER be called with a value less than 2 us (infinite loop) <<<<

; IN: r18 = number of microseconds to sleep
; SCRATCHED: r0,r1,r17,r27:r26
;
; Total execution time: 
; +1 cycles for caller having to load the R18 register with time to wait
; +4 cycles for CALL invoking this method
; +5 cycles for calculating cycle count
; +4 cycles for RET 
; =========
usleep:
          ldi r17 , cycles_per_us ; 1 cycles
          mul r18 , r17 ; 1 cycle , result is in r1:r0     
          movw r27:r26 , r1:r0 ; 1 cycle
          sbiw r27:r26,14  ; 2 cycles , adjust for cycles spent invoking this method + preparation  
.loop     sbiw r27:r26,4 ; 2 cycles , subtract 4 cycles per loop iteration      
	brpl loop ; 2 cycles, 1 cycle if branch not taken
	ret ; 4 cycles