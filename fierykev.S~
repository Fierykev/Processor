.global main
main:
b start
startup:
    li 17, 'A'
    blr
printIncChar:
    addi 17, 17, 1
    mr 3, 17
    b printChar
printChar:
    li 0, 0
    sc
    blr
printHex:
    li 0, 2
    sc
    blr
exit:
    li 0, 1
    sc
start:
    bl startup

    # check last bit in condition register is blank
    bcl 2, 31, exit

    # check ctr decrement works
    bcl 16, 2, printIncChar
    bcl 18, 2, exit

    # greater than / less then / eq / ov (also tests ld)
    li 2, blueExorcist-main
    ld 3, 0(2)
    mr 2, 3
    li 5, 1
    li 4, 2
    bl printHex
    addi 3, 2, 2 # check addi does not change overflow

    # now start real checks
    add. 3, 2, 5
    addo 3, 2, 5
    bso exit

    addo 3, 2, 5
    or. 3, 15, 16 # check for or update
    bns exit

    # branch less than test
    add. 3, 2, 4
    bltl printIncChar
    bgt exit
    beq exit
    bns exit

    # branch greater than test
    addo. 3, 5, 5
    bgtl printIncChar
    blt exit
    beq exit
    bns exit

    # branch equal to test
    li 4, -1
    addo. 3, 5, 4
    beql printIncChar
    blt exit
    bgt exit
    bns exit

    # ldu test
    li 2, 8
    ldu 3, 4(2)
    bl printHex;

    addi 3, 2, 53
    bl printChar

    # bclr test
    li 5, 9
    bl loop

loop:
    addi 5, 5, -1
    addi 3, 5, '1'
    addo. 6, 5, 5
    ldu 8, 4(7) # this is here to slow down the test for people
    ldu 8, 4(7)
    li 0, 0
    sc
    bclr 0b00100, 2

    # fake ldu
    li 0, '@'
.data
    .int 0xe8600005 # load with zero as RA
.text
    mr 3, 0
    bl printChar
.data
    .int 0xe8618005 # load with RA = RT (both are 3)
.text
    bl printChar

    # fake sc
.data
    .int 0x44000000 # no 1 at the end
.text

    # fake ld
    li 3, '&'
.data
    .int 0xe8620008
.text
    bl printChar

    # fake bclr
    bl 1f

    b exit
1:
    .int 0x4380030
    bl printIncChar

# NEW TESTS FOR PA
    li 5, 16
    .align 3
    # std overwrite with ld's
stdStart:
    b 1f
    li 0, 2 # 8
    li 3, 'a'
1:
    ld 3, (subData-main - 16)(5) # 16
    li 4, 'b'
    std 3, (sub-main - 16)(5) # 24
sub:
    li 3, 'c'
    li 3, 'd' # 32
sub2:
    li 3, 'm'
    li 3, 's' # 40

    ld 3, sub2-main(0)
    b 1f # 48
subData:
    ld 3, stdStart-main(0)
    std 3,  sub2-main(0)
1:
    bl printHex

    # MTCRF test
    ld 5, MTCRFData-main(0)
    mtcrf 0x80, 5

    bltl exit
    bgtl printIncChar
    beql printIncChar
    bsol printIncChar

    li 3, 'N'
    bl printChar

    # MTSPR tests
    # Link register test loop
    li 5, 0

    .align 3
loopStart:
    li 4, loopStart-main
    mtspr 8, 4

    nop
    std 5, loopStart-main(5)

    li 3, 'Z'
    sc

    li 5, 24
    blr

    .align 3

    # move data to xer and check SO (should be false)
    li 4, 0
    mtxer 4
    or. 4, 4, 4 # update condition reg

    bso exit
    blt exit
    bgt exit
    bne exit
    bl printIncChar

    # move data to the counter and loop
    li 4, 5
    mtctr 4

countLoop:
    bl printIncChar
    bca 0b10000, 2, countLoop-main

    # overwrite branch with branch
    ld 3, printData-main(0)
    std 3, exitBlock-main(0) 

   #.align 3
exitBlock:
    b exit
    b exit

    b 1f

    .align 3
printData:
    bla printIncChar-main
    bla printIncChar-main

1:

# mtcr no op
    li 3, 52
.data
    .int 0x7c7ff120
.text

    mfcr 3
    bl printChar

    b exit
    .global entry
entry :
    .quad main
    .quad data
    .quad 0
    .align 3
blueExorcist:
        .quad 0x7fffffffffffffff
MTCRFData:
        .quad 0x000000007542FFEE
