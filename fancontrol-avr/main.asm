; Copyright 2020 tobias.gierke@code-sourcery.de
; 
; Licensed under the Apache License, Version 2.0 (the "License");
; you may not use this file except in compliance with the License.
; You may obtain a copy of the License at
; 
;     http://www.apache.org/licenses/LICENSE-2.0
; 
; Unless required by applicable law or agreed to in writing, software
; distributed under the License is distributed on an "AS IS" BASIS,
; WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
; See the License for the specific language governing permissions and
; limitations under the License.

#include "m328Pdef.inc"

.equ freq = 16000000 ; Hz
.equ cycles_per_ms = freq / 1000

.equ serial_rate = 25 ; 38400 bps @ 16 Mhz
  
.equ rx_buffer_size = 32
.equ tmp_buffer_size = 32
  
.equ pwm_frequency = 0x140 ; yields 25khz PWM on 16 Mhz CPU, depends on prescaler setting & CPU frequency
  
; timeout after which we're going
; to switch the fan back to full speed
; unless we received a 'set fan speed' command
; within this timeframe
.equ watchdog_timer_value = 183 ; let TC0 count to 183 * 255 at (16 Mhz/1024) rate -> roughly 3 seconds
  
#define BUZZER_ENABLED
  
// #define PC_WATCHDOG
#define CRASH_WATCHDOG
  
.equ SRAM_END = 0x8ff
  
; onboard LED is on PB5
#define ONBOARD_LED 5
  
#define ONBOARD_LED_ON sbi PORTB,5
#define ONBOARD_LED_OFF cbi PORTB,5
  
jmp init  ; RESET
jmp onirq ; INT0 - ext IRQ 0
jmp onirq ; INT1 - ext IRQ 1
jmp onirq ; PCINT0 - pin change IRQ 
jmp onirq ; PCINT1 - pin change IRQ
jmp onirq ; PCINT2 - pin change IRQ
jmp guardian_irq ; WDT - watchdog IRQ
jmp onirq ; TIMER2_COMPA - timer/counter 2 compare match A
jmp onirq ; TIMER2_COMPB - timer/counter 2 compare match B
jmp onirq ; TIMER2_OVF - timer/counter 2 overflow
jmp onirq ; TIMER1_CAPT - timer/counter 1 capture event
jmp onirq ; TIMER1_COMPA 
jmp onirq ; TIMER1_COMPB
jmp onirq ; TIMER1_OVF
jmp onirq ; TIMER0_COMPA
jmp onirq ; TIMER0_COMPB
jmp watchdog_irq ; TIMER0_OVF
jmp onirq ; STC - serial transfer complete (SPI)
jmp uart_receive ; USUART Rx complete
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
    eor r16,r16
    out 0x3f,r16
; initialize stack pointer  
    ldi r16,HIGH(SRAM_END) ; 0x08  
    out 0x3e,r16  ; SPH
    ldi r16,LOW(SRAM_END) ; 0xff  
    out 0x3d,r16  ; SPL  
  
    sbi DDRB,ONBOARD_LED ; enable output    
          
    rcall buzz
  
    cli    
       
    rcall enable_pwm_out ; will set fan speed to 100%
    rcall uart_init
#ifdef CRASH_WATCHDOG  
    rcall start_guardian_timer ; will reset the unit if the main loop doesn't execute in <2 seconds
#endif  
    sei
  
; ===== MAIN LOOP
.mainloop
    wdr ; ping watchdog timer so the system doesn't reset
    lds r16, in_ready
    tst r16
    breq mainloop      
  
; UART input buffer contains a line, process it !
    rcall tokenize_input ; replaces any byte below 33 with 0x00
  
; R19 now contains the number of tokens
    cpi r19,1
    brlo syntax_error ; need at least one token (the command to execute)
  
; ====== check for 'STATUS' command =====  
    ldi ZH, HIGH(req_cmd_status)
    ldi ZL, LOW(req_cmd_status)
    rcall str_icmp
; R16 holds result, 0x00 = equals, 0xff = not equals
    tst r16
    brne check_set
; received 'STATUS' command  
    cpi r19,1
    ldi ZH, HIGH(resp_cmd_status)
    ldi ZL, LOW(resp_cmd_status)
    rcall tx_program_string
    rjmp processing_done

; ====== SET command =====
.check_set
    ldi ZH, HIGH(req_cmd_set)
    ldi ZL, LOW(req_cmd_set)
    rcall str_icmp
; R16 holds result, 0x00 = equals, 0xff = not equals
    tst r16
    brne syntax_error
    cpi r19,2
    brne syntax_error
; received a valid 'SET' command (2 tokens, the command itself and the desired speed)
    ldi ZH, HIGH(in_buffer)
    ldi ZL, LOW(in_buffer)
    ldi r16,1
    rcall find_token
          
; Z now points to number (hopefully)  
    rcall parse_number
    tst r16
    brmi syntax_error
    cpi r16,101
    brsh syntax_error
  
; set new fan speed  
    rcall set_fan_speed
  
    lds r16, watchdog_started
    tst r16
    brne reset_watchdog
; watchdog never started, start it
#ifdef PC_WATCHDOG  
    rcall start_watchdog
#endif  
    rjmp cont
; watchdog running, reset it  
.reset_watchdog
    ldi r16,watchdog_timer_value
    sts watchdog_timeout, r16
.cont      
    ldi ZH, HIGH(resp_cmd_set)
    ldi ZL, LOW(resp_cmd_set)
    rcall tx_program_string ; print 'ok - speed set to '
    
    lds r16, fan_speed
    ldi ZH, HIGH(tmp_buffer)
    ldi ZL, LOW(tmp_buffer)
    rcall print_number
    
    ldi ZH, HIGH(tmp_buffer)
    ldi ZL, LOW(tmp_buffer)
    rcall tx_data_string ; print number
          
    ldi r16,10
    rcall tx_byte
  
    rjmp processing_done  
 
; we received something unexpected
.syntax_error
    ldi ZH, HIGH(resp_error)
    ldi ZL, LOW(resp_error)
    rcall tx_program_string
    rjmp processing_done
  
.processing_done
    clr r16
    sts in_write_ptr, r16
; in_ready needs to be cleared LAST
; as the UART RX IRQ routine might start
; writing to the input buffer (and using the write ptr)
; right after it gets cleared  
    sts in_ready, r16
    rjmp mainloop 
    
onirq:
    rcall blink
    rjmp onirq
  
; === enable guardian timer ===
; IRQs need to be turned OFF when invoking this method
; === 
start_guardian_timer:
; Reset Watchdog Timer
    wdr
; Start timed sequence
    lds r16, WDTCSR
    ori r16, (1<<WDCE) | (1<<WDE)
    sts WDTCSR, r16
; -- Got four cycles to set the new values from here -
; Set new prescaler(time-out) value = 64K cycles (~2 s)
    ldi r16, (1<<WDE) | (1<<WDIE) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0)
    sts WDTCSR, r16
; -- Finished setting new values, used 2 cycles -
    ret
  
; ====
; Triggered by AVR watchdog timer
; if the main loop is not getting executed
; within 2 seconds
; ====
guardian_irq:
    rcall buzz
    rcall buzz
    rcall buzz
    reti  
  
; ====
; Slowly blinks the onboard led once
; ==== 
blink:
    cbi PORTB,ONBOARD_LED  
    rcall delay    
    sbi PORTB,ONBOARD_LED
    rcall delay    
    cbi PORTB,ONBOARD_LED      
    ret 
  
; ================
; Sounds 3 beeps (output a 440 Hz square wave via TC2 to drive a buzzer at PB3 aka D3 on Arduino)
; ===============
.equ buzz_freq = 440
.equ buzz_time = (freq/256) / buzz_freq - 1    
buzz:
  
#ifndef BUZZER_ENABLED
  ret
#endif
  push r16
 ;wgm02..0 = 7 (fast pwm, top = ocr0a)
  ;cs02..0 = 4 (N=256)
  ;com0b1..0 = 2 (clear on compare match, set at bottom)
  ldi r16, 1<<COM2B1 | 1<<WGM21 | 1<<WGM20
  sts TCCR2A, r16
  
  ldi r16, 1<<WGM22 | 1<<CS22 | 1<<CS21
  sts TCCR2B, r16
  
; set PWM frequency
  ldi r16, buzz_time
  sts OCR2A, r16
  
; 50% duty cycle
  ldi r16, buzz_time/2
  sts OCR2B, r16  
  
  ; output at D3  
  sbi DDRD, 3
  
  ldi r16, 3
.loop
  rcall delay   
  cbi DDRD, 3  
  dec r16
  breq leave  
  rjmp loop
.leave    
  clr r16
  sts TCCR2A,r16
  pop r16
  ret  
  
; =====
; Start serial control commands watchdog timer.
;
; This timer is reset every time a "set fan speed" command is received
; via the serial bus.
; if no command is for roughly 3 seconds (watchdog_timer_value),
; an IRQ will be triggered that sounds the buzzer 
; and then switches the fan to 100%
;
; SCRATCHED: r16
; =====
start_watchdog:
    
    ldi r16, watchdog_timer_value
    sts watchdog_timeout, r16
  
    ldi r16,1
    sts watchdog_started, r16
  
; setup TC0 8 bit timer
    ldi r16,1<<TOIE0 ; enable overflow IRQ
    sts TIMSK0, r16    
    ldi r16, (1<<CS02) | (1<<CS00) ; start timer by setting prescaler = 1024
    out  TCCR0B, r16    
    ret

; ==== invoked when TC0 overflows (= no "set fan speed" command received for approx. 3 seconds)
watchdog_irq:
    push r16
    in r16, SREG
    push r16
; actual IRQ routine starts here  
    lds r16, watchdog_timeout
    dec r16
    sts watchdog_timeout, r16
    brne leave
; timeout elapsed,
; set fan to 100%
; and stop timer
    ldi r16, 100
    rcall set_fan_speed 
  
    ldi r16,0
    sts watchdog_started, r16 ; ; mark watchdog as stopped 
    out TCCR0B, r16 ; disable timer (set clock source bits to %000)
    sts TIMSK0, r16 ; disable overflow IRQ
  
    rcall buzz
    rcall buzz
  
; leave IRQ  
.leave  
    pop r16
    out SREG, r16
    pop r16
    reti
  
; ===
; write number as decimal string to output buffer
; INPUT: r16 - number to print
; INPUT: Z - pointer to output buffer (DATA SEGMENT)
; SCRATCH: r16, Z (will point to byte after last digit)
print_number:
  push r17
  push r18
  
  ldi r18, '0'    
  mov r17, r16 ; backup
; / 100
  rcall div_by_ten
  rcall div_by_ten
  add r16, r18 ; + '0'
  st Z+,r16
  sub r16,r18 ; undo + '0'
  rcall mul_by_ten
  rcall mul_by_ten  
  sub r17, r16
; / 10
  mov r16, r17 
  rcall div_by_ten
  add r16, r18 ; + '0'
  st Z+,r16
  sub r16,r18 ; undo + '0'
  rcall mul_by_ten  
  sub r17, r16  
; / 1  
  add r17, r18 ; + '0'  
  st Z+,r17     
; return
  clr r18
  st Z+,r18
  pop r18
  pop r17
  ret
  
; ======
; find start of n-th token in an array.
; tokens are separated by one or more zero bytes
; INPUT: Z - start of token buffer
;        r16 - token index
; SCRATCH: r16
; OUTPUT: Z - points to n-th token
; =====
find_token:
  push r17
  tst r16
  breq return
; skip non-zero bytes (if any)
.skip_non_zero
  ld r17,Z
  tst r17
  breq skip_zero
  adiw Z,1
  rjmp skip_non_zero
.skip_zero
  ld r17, Z
  tst r17
  brne non_zero
  adiw Z,1
  rjmp skip_zero
.non_zero
  dec r16
  brne skip_non_zero
.return
  pop r17
  ret

; ========
; Parse a zero-terminated string as decimal number (at most 3 digits are supported)
; INPUT: Z - pointer to string (DATA SEGMENT)
; SCRATCHED: r18,r19,r20,Z
; RESULT: r16
; ========
parse_number:   
   ldi r19,4 ; 4 characters to process  
   clr r20 ; result
.loop
   ld r18,Z+
   tst r18
   breq leave
 
   mov r16, r20
   rcall mul_by_ten ; r16 = r16 * 10
   mov r20, r16

   subi r18,'0'
   brmi error
   cpi r18,10
   brsh error
  
   add r20,r18
   dec r19
   brne loop
   rjmp error   
.leave
   mov r16, r20
   ret
.error  
   ldi r16, 0xff
   ret
 
; =======
; Multiply by 10.
; INPUT: r16
; OUTPUT: r16 = r16 * 10
; ========
mul_by_ten:
      push r17
      mov r17, r16
; *8  
      lsl r16
      lsl r16
      lsl r16
; *2  
      lsl r17
;
      add r16,r17
      pop r17
      ret
  
; =======
; Divide by 10.
; INPUT: r16,r17
; OUTPUT: r16 = r16/r17
; ========
div_by_ten:
  push r17
  ldi r17,10
  rcall div8u
  pop r17
  ret
  
; ======
; divide r16/r17 (unsigned 8 bit division)
; INPUT: r16, r17
; SCRATCH: r17
; OUTPUT: r16 
; =====
div8u:  
  
.def  drem8u  = r15    ;remainder
.def  dres8u  = r16    ;result
.def  dd8u  = r16    ;dividend
.def  dv8u  = r17    ;divisor
.def  dcnt8u  = r18    ;loop counter  
  
  push r15
  push r18
  
  sub  drem8u,drem8u  ;clear remainder and carry
  ldi  dcnt8u,9  ;init loop counter
d8u_1:  rol  dd8u    ;shift left dividend
  dec  dcnt8u    ;decrement counter
  brne  d8u_2    ;if done
  
  pop r18
  pop r15
  ret      ;    return
  
d8u_2:  
  rol  drem8u    ;shift dividend into remainder
  sub  drem8u,dv8u  ;remainder = remainder - divisor
  brcc  d8u_3    ;if result negative
  add  drem8u,dv8u  ; restore remainder
  clc      ;    clear carry to be shifted into result
  rjmp  d8u_1    ;else
d8u_3:  sec      ;    set carry to be shifted into result
  rjmp  d8u_1 
 
; =====
; Delay for 1 second
; =====
delay:
    push YH
    push YL
; sleep 1000 milliseconds  
    ldi YH,HIGH(250)
    ldi YL,LOW(250)
    rcall sleep
    pop YL
    pop YH
    ret

; ======
; Sleep for a given number of milliseconds
; INPUT: YH,YL - time in milliseconds
; ======
sleep:

  push ZH ; 2 cycles
  push ZL ; 2 cycles
    
; outer loop, ~1 ms per execution  
.loop0  
  ldi ZH,HIGH((cycles_per_ms/4)-4) ; (hint: 4 cycles per loop iteration and 4 extra cycles every millisecond from the 2nd outer loop)
  ldi ZL,LOW((cycles_per_ms/4)-4) ; 1 cycle      
; inner loop that runs for ~1000 ms  
.loop1
  sbiw Z,1 ; 2 cycles
  brne  loop1 ; 1 cycle if condition is false, 2 cycles if condition is true
  
  sbiw Y,1 ; 2 cycles
  brne loop0 ; 1 cycle if condition is false, 2 cycles if condition is true    
    
  pop ZL ; 2 cycles
  pop ZH ; 2 cycles

  ret ; 4 cycles
  
; ==========================
; start generating PWM signal for fan on OC1B (aka D10 on Arduino) 
; ==========================
enable_pwm_out:      
    sbi      DDRB, 2 ; switch PB2 (OC1B) to output
    
    ldi      r17, (1<<COM1B1) ;Clear OC1B on compare match when up-counting, set OC1B on compare match when down-counting
    sts      TCCR1A, r17

    ldi      r17, (1<<WGM13) | 1<<CS10  ; PWM Phase & Frequency correct, Prescaler = x1 , 
    sts      TCCR1B, r17   
; set PWM frequency
; this value works for a 16 Mhz MCU,
; Prescaling & this value needs to be adjusted for 
; lower frequencies
    ldi      r17, HIGH(pwm_frequency)
    ldi      r16, LOW(pwm_frequency)
    sts      ICR1H, r17
    sts      ICR1L, r16
; let fan run at 100% when this fan controller boots up
    ldi r16,100
    rjmp set_fan_speed
    
; ======
; Set fan speed.
; INPUT: r16 - fan speed in percent (0..100)
; ======
set_fan_speed:
     
    push r14
    push r15
    push r16
    push r17
    push r18
    push r19
    push r20
    push r21
    push r22  
   
    clr r17  ; clear high byte
    sts fan_speed, r16
; max(pwm timer value) * fan_speed
; Multiplies the two 16-bit register variables r17:r16 and r19:r18 (unsigned)
    ldi r19,HIGH(pwm_frequency)
    ldi r18,LOW(pwm_frequency)
    rcall mul_16u
; RESULT: r21:r20:r19:r18
    mov r17,r19
    mov r16,r18      
; divided by 100
; Divides r17:r16 by r19:r18 (16/16 bit unsigned)  
    clr r19
    ldi r18, 100
    rcall div16u
; result in r17:r16
    sts OCR1BH, r17
    sts OCR1BL, r16
  
    pop r22
    pop r21
    pop r20
    pop r19
    pop r18
    pop r17
    pop r16
    pop r15
    pop r14
  
    ret  
  
; =====
; Initialize UART
; to 8N1 @ 9600 bits/s
; =====  
uart_init:
    clr r16
    sts in_overflow, r16
    sts in_ready, r16
    sts in_write_ptr, r16
; Set baud rate
    ldi r17,HIGH(serial_rate)
    ldi r16,LOW(serial_rate)
    sts UBRR0H, r17
    sts UBRR0L, r16
; Enable receiver and transmitter
;   UCSZ02    UCSZ01   UCSZ00
;      1         1        1           => 8 data bits
  
    ldi r16, (1<<RXCIE0) |(1<<RXEN0)|(1<<TXEN0) | (1<<UCSZ02)
    sts UCSR0B,r16
; Set frame format: 8 data, no parity, 1 stop bit
    ldi r16, (1<<UCSZ01) |  (1<<UCSZ00)
    sts UCSR0C,r16
    ret
  
; =====
; UART receive interrupt
; =====
uart_receive:
; save registers
    push YH
    push YL
    push r18  
    push r17
    push r16
    in r16, SREG
    push r16
; read serial data  
    lds r18, UDR0   
; check whether we're already done processing
 ; any previous input we might have received  
    lds r16, in_ready
    tst r16
    brne input_lost ; not done processing input yet
; check whether the previous byte was lost,
; discard any bytes until we receive a linefeed if this was the case
    lds r16, in_overflow
    brne input_lost
; make sure we're not overflowing the input buffer
    lds r17, in_write_ptr    
    cpi r17, rx_buffer_size
    breq input_lost    
; load ptr to start of input buffer  
    ldi YH, HIGH(in_buffer)
    ldi YL, LOW(in_buffer)
; add write ptr to input buffer ptr    
    add YL,r17
    clr r16
    adc YH,r16
; store received byte in input buffer        
    st Y,r18
; inc write ptr
    inc r17
    sts in_write_ptr, r17
; linefeed received ?
    cpi r18, 0x0a 
    breq got_linefeed
    cpi r18,0x0d
    breq got_linefeed
    rjmp leave
.got_linefeed  
; linefeed received, signal main thread 
; that in_buffer is ready for processing
    ldi r16,0x01
    sts in_ready,r16
    rjmp leave

; received byte in r18, we were unable to
; write the received byte to in_buffer
.input_lost 
    ldi r16,1
    sts in_overflow, r16
    cpi r18, 0x0a
    brne leave
; buffer is full but we finally received a linefeed,
; start capturing again  
    clr r16
    sts in_write_ptr, r16
    sts in_overflow, r16
    rjmp leave
  
; ==== return from IRQ =====  
.leave
    pop r16
    out SREG, r16
    pop r16
    pop r17 
    pop r18 
    pop YL
    pop YH
    reti
  
; ==========
; compare serial input buffer 
; case-insensitive against a constant
; string stored in PROGRAM MEMORY.
;
; strings must be terminated by zero bytes
; IN: Z - ptr to zero-terminated string in PROGRAM MEMORY 
; SCRATCH: Z
; RESULT: r16 - 0 (equals), 0xff (not equal)
; ==========
str_icmp:
    push r17
    push YH
    push YL
    ldi YH,HIGH(in_buffer)
    ldi YL,LOW(in_buffer)
.loop
    lpm r16,Z+
    ld r17,Y+
    cp r16,r17
    brne fix_case
; same character
    tst r16   
    brne loop
; both strings are equal
    clr r16
    rjmp leave  
.fix_case
  ; try again, case-insensitive
    ori r16,1<<5
    ori r17,1<<5
    cp r16, r17
    breq loop
; mismatch
    ldi r16,0xff
.leave
    pop YL
    pop YH
    pop r17  
    ret  
  
; =====
; overwrite all characters 
; inside the UART input buffer that are 
; below ASCII 33 with zero bytes
; SCRATCH: r16, r17, r18, r19, r20, Z
; RETURN: r19 - number of tokens
; =====
tokenize_input:
  clr r19 ; number of tokens  
  lds r16, in_write_ptr
  tst r16
  breq leave ; no data to process    
  ldi ZH, HIGH(in_buffer)
  ldi ZL, LOW(in_buffer)
  clr r17 ; 0 byte used for overwriting
  clr r20 ; flag. previous byte got replaced
.loop
  ld r18,Z
  cpi r18,33
  brsh ok ; character is >= 33
; overwrite with zero byte
  st Z,r17
  tst r20 ; did the previous byte also get overwritten?
  brne multiple_bytes ; yes, don't increment token count
  inc r19 ; +1 token
.multiple_bytes  
  ldi r20,1 ; set "previous byte replaced" flag
  rjmp ok2
.ok 
  clr r20 ; clear "previous byte replaced" flag
.ok2
  adiw Z,1 ; advance to next input byte
  dec r16
  brne loop
.leave  
  ret
  
; ====
; send zero-terminated string stored in PROGRAM MEMORY via the UART
; INPUT: Z - ptr to program memory
; SCRATCH: Z
; ====
tx_program_string:
  push r16
  push r17
  
.loop
  lpm r16,Z+
  tst r16
  breq return
.wait_tx_buf_empty  
  lds r17, UCSR0A
  sbrs r17, UDRE0
  rjmp wait_tx_buf_empty      
  sts UDR0,r16 ; send byte  
  rjmp loop
  
.return
  pop r17
  pop r16
  ret
    
; ====
; send zero-terminated string stored in DATA MEMORY via the UART
; INPUT: Z - ptr to program memory
; SCRATCH: Z
; ====
tx_data_string:
  push r16
  push r17
  
.loop
  ld r16,Z+
  tst r16
  breq return
.wait_tx_buf_empty  
  lds r17, UCSR0A
  sbrs r17, UDRE0
  rjmp wait_tx_buf_empty      
  sts UDR0,r16 ; send byte  
  rjmp loop
  
.return
  pop r17
  pop r16
  ret  
  
; ====
; sends the byte in r16 via the UART
; ====  
tx_byte:
  push r17
.wait_tx_buf_empty  
  lds r17, UCSR0A
  sbrs r17, UDRE0
  rjmp wait_tx_buf_empty      
  sts UDR0,r16 ; send byte  
  pop r17
  ret  
  
; ==================
; Divides r17:r16 by r19:r18 (16/16 bit unsigned)
;
;* RESULT: r17:r16 and r15:r14 (remainder)
; SCRATCHED: r18,r19,r20
; ==================
  
.def  drem16uL=r14
.def  drem16uH=r15
.def  dres16uL=r16
.def  dres16uH=r17
  
.def  dd16uL  =r16
.def  dd16uH  =r17
.def  dv16uL  =r18
.def  dv16uH  =r19
.def  dcnt16u  =r20

div16u:  
  clr  drem16uL  ;clear remainder Low byte
  sub  drem16uH,drem16uH;clear remainder High byte and carry
  ldi  dcnt16u,17  ;init loop counter
.d16u_1
  rol  dd16uL    ;shift left dividend
  rol  dd16uH
  dec  dcnt16u    ;decrement counter
  brne  d16u_2    ;if done
  ret      ;    return
.d16u_2
  rol  drem16uL  ;shift dividend into remainder
  rol  drem16uH
  sub  drem16uL,dv16uL  ;remainder = remainder - divisor
  sbc  drem16uH,dv16uH  ;
  brcc  d16u_3    ;if result negative
  add  drem16uL,dv16uL  ;    restore remainder
  adc  drem16uH,dv16uH
  clc      ;    clear carry to be shifted into result
  rjmp  d16u_1    ;else
.d16u_3
  sec      ;    set carry to be shifted into result
  rjmp  d16u_1  
  
; =================
; Multiplies the two 16-bit register variables r17:r16 and r19:r18 (unsigned)
; RESULT: r21:r20:r19:r18
; SCRATCHED: r22
; ================

.def  mc16uL  =r16    ;multiplicand low byte
.def  mc16uH  =r17    ;multiplicand high byte
.def  mp16uL  =r18    ;multiplier low byte
.def  mp16uH  =r19    ;multiplier high byte
.def  m16u0  =r18    ;result byte 0 (LSB)
.def  m16u1  =r19    ;result byte 1
.def  m16u2  =r20    ;result byte 2
.def  m16u3  =r21    ;result byte 3 (MSB)
.def  mcnt16u  =r22    ;loop counter

mul_16u:  
  clr  m16u3    ;clear 2 highest bytes of result
  clr  m16u2
  ldi  mcnt16u,16  ;init loop counter
  lsr  mp16uH
  ror  mp16uL

.m16u_1
  brcc  noad8    ;if bit 0 of multiplier set
  add  m16u2,mc16uL  ;add multiplicand Low to byte 2 of res
  adc  m16u3,mc16uH  ;add multiplicand high to byte 3 of res
.noad8
  ror  m16u3    ;shift right result byte 3
  ror  m16u2    ;rotate right result byte 2
  ror  m16u1    ;rotate result byte 1 and multiplier High
  ror  m16u0    ;rotate result byte 0 and multiplier Low
  dec  mcnt16u    ;decrement loop counter
  brne  m16u_1    ;if not done, loop more
  ret  

req_cmd_set: .db "set",0
resp_cmd_set: .db "ok - speed ",0
  
req_cmd_status: .db "status",0
resp_cmd_status: .db "ok",10,0
  
resp_error: .db "error",10,0 
  
  .dseg
  
; a watchdog timer will be started and this flag set to '1' 
;  after we've received at least one 
; 'set fan speed' command via USB  
  watchdog_started: .byte 1
  
; timeout counter, if it
; reaches zero without receiving any 
; 'set fan_speed' command, we'll
; force the fan to spin at 100%
; to protect the hardware should
; the controlling daemon process
; die unexpectedly  
  watchdog_timeout: .byte 1
  
; fan speed in percent (0..100)
  fan_speed: .byte 1
  
; indicates that we received a line via the UART 
; that now sits in the in_buffer and awaits processing  
  in_ready: .byte 1  
; used by IRQ UART RX routine to track where the
; next byte needs to be written
  in_write_ptr: .byte 1
; indicates that we received rx_buffer_size bytes
; without a linefeed, swallow all received bytes until we do  
  in_overflow: .byte 1  
  
; buffer where the UART RX IRQ will store received bytes
 in_buffer: .byte rx_buffer_size  
 tmp_buffer: .byte tmp_buffer_size
  