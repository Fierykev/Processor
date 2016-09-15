        .section ".text"
        .global main
        .section ".opd", "aw"
        .global entry entry:
        .quad main, .TOC.@tocbase,0
        .previous
        .type entry, @function
        .section ".text" main:
        li %r3, 'a'
        li %r0, 2
        sc mem_read_write:
        ld %r3, mydata_first-main (0)
        sc
        li %r4, 0xab
        std %r4, mydata_first-main (0)
        ld %r3, mydata_first-main (0)
        sc
        li %r4, 0xbc
        std %r4, mydata_first-main (0)
        nop
        ld %r3, mydata_first-main (0)
        sc
        li %r4, 0xdf
        std %r4, mydata_first-main (0)
        nop
        nop
        ld %r3, mydata_first-main (0)
        sc
        .align 3 smc:
        li %r4, 0
        li %r3, 0x96
        std %r4, next_inst-main (0)
        .align 3 next_inst:
        sc # will be overwritten by nop
        nop smc_2:
        ld %r4, mydata_second - main (0)
        std %r4, next_inst_2-main (0)
        nop
        .align 3 next_inst_2:
        b ex # will be overwritten
        nop part_two:
        ld %r3, mydata_first-main (0)
        std %r4, mydata_first-main (0)
        add %r3, %r3, %r3
        sc
        b part_three part_three:
        li %r4, 0
        std %r4, part_three_exit - main(0)
        .align 3 part_three_exit:
        sc
        b main
        b part_four
        .align 3 mydata_first:
        .quad 0
        .align 3 mydata_second:
        b mydata_second_end mydata_second_end:
        nop
        b main part_four:
        li %r3, part_five-main
        mtlr %r3
        blr
        b ex part_five:
        bl part_five_2 part_five_2:
        sc
        mflr %r3
        sc
        bgt ex
        li %r2, -1
        mtcr %r2
        bgt part_six
        b ex part_six:
        sc
        b part_seven part_seven:
        li %r3, 10
        mtctr %r3 part_seven_loop:
        mfctr %r3
        sc
        mtctr %r3
        bdnz part_seven_loop part_eight:
        ld %r4, bignum-main (0)
        mr %r5, %r4
        addo. %r4, %r5, %r4
        bso part_eight_overflow
        b ex part_eight_overflow:
        li %r1, 0
        mtcr %r1
        bso ex
        li %r3, 0x98
        sc
        li %r3, 1
        mtctr %r3
        bdz part_nine
        b ex part_nine:
        li %r3, 12
        std %r3, mydata_first - main (0)
        ld %r4, mydata_first - main (0)
        li %r3, 0
        mr %r3, %r4
        sc
        li %r3, 13
        std %r3, mydata_first - main (0)
        nop
        ld %r4, mydata_first - main (0)
        li %r3, 0
        mr %r3, %r4
        sc
        bl subroutine
        b ex subroutine:
        li %r4, part_ten - main
        mtlr %r4
        blr
        b ex part_ten:
        li %r1, 2
        li %r2, 3
        add. %r1, %r2, %r1
        bgt part_ten_jump
        b ex part_ten_jump:
        li %r1, 8
        ld %r3, bignum_32 - main - 8(%r1)
        mtcr %r3
        blt ex
        bgt part_ten_jump_2
        b ex part_ten_jump_2:
        sc
        b part_eleven part_eleven:
        li %r1, 0
        mtcr %r1
        mtxer %r1
        bso ex
        li %r1, -1
        mtxer %r1
        bso ex
        addo. %r1, %r1, %r1
        bso part_twelve
        b ex part_twelve:
        li %r3, 14
        sc
        mfxer %r3
        sc part_thirteen:
        li %r1, 1
        or. %r1, %r1, %r1
        li %r0, 0
        .int 0xF81FF921 # mtcr %r0, with bit 11 set to 1
        bgt part_fourteen
        b ex part_fourteen:
        li %r0, 2
        sc
        li %r3, 32
        addi %r3, %r3, 10
        li %r2, 64
        sc
        ld %r3, 0(0)
        add %r3, %r3, %r3
        li %r2, 64
        li %r0, 2
        sc
        add %r1, %r2, %r3
        add %r3, %r1, %r2 # Read after write
        sc
        add %r3, %r2, %r1
        add %r3, %r2, %r1 # Write after write, also testing writing the same register on 
both ports
        sc
        add %r1, %r3, %r2
        add %r3, %r2, %r1 # Write after read
        sc
        b part_fifteen part_fifteen:
        li %r5, (part_fifteen - main)/8
        mtctr %r5 clear_loop:
        std %r31, 0(%r5)
        addi %r5, %r5, 8
        bdnz clear_loop
        li %r3, 1
        li %r2, -8
        li %r4, number_positive1-main
        li %r5, (end_of_program-main)/8 # counter
        mtctr %r5
        li %r6, -1 slow_loop:
        add. %r5, %r6, %r5 slow_loop_1:
        mfctr %r3
        sc
        bdz slow_loop_end
        ldu %r3, 8(%r2)
        add %r3, %r3, %r3
        sc
        b slow_loop_1 slow_loop_end:
        li %r3, 'O'
        bl dispChar
        li %r0, 0 part_sixteen:
        li %r1, 0x123
        mtxer %r1
        bl dispNum
        mfxer %r3
        bl dispNum
        addi %r1, %r1, 0x123
        mtlr %r1
        bl dispNum
        mflr %r3
        bl dispNum
        li %r1, 0xabc
        mtctr %r1
        bl dispNum
        mfctr %r3
        bl dispNum old_test:
        li %r3, 32
        addi %r3, %r3, 10
        li %r2, 64
        bl dispNum
        ld %r3, 0(0)
        add %r3, %r3, %r3
        li %r2, 64
        li %r0, 2
        bl dispNum
        add %r1, %r2, %r3
        add %r3, %r1, %r2 # Read after write
        li %r0, 1
        bl dispNum
        add %r3, %r2, %r1
        add %r3, %r2, %r1 # Write after write, also testing writing the same register on 
both ports
        bl dispNum
        add %r1, %r3, %r2
        add %r3, %r2, %r1 # Write after read
        sc
        li %r1, 0
        mtcr %r1
        mtxer %r1
        mtlr %r1 older_test:
        li %r1, 1234
        li %r2, -12345
        add. %r1, %r2, %r1
        beq ex
        bgt ex
        blt condition_chk_2 condition_chk_2:
        li %r1, 12345
        li %r2, -2
        add. %r1, %r2, %r1
        beq ex
        blt ex
        bgt condition_chk_3 condition_chk_3:
        li %r1, 300
        li %r2, -300
        add. %r1, %r1, %r2
        blt ex
        bgt ex
        beq overflow_chk overflow_chk:
        li %r10, bignum1-main
        ld %r1, 0(%r10)
        ld %r2, 0(%r10)
        add %r1, %r1, %r2 # SO should not be set
        bso ex
        or. %r1, %r1, %r1
        bso ex
        ld %r1, 0(%r10)
        addo %r1, %r1, %r2 # SO should be set
        bso ex # cr0[3] should not be set
        or. %r1, %r1, %r1
        bso next # SO should now be set
        li %r3, 'O'
        bl dispChar
        b ex next:
        oris %r3, %r3, 30 # invalid instruction
        bl dispNum
        li %r3, 4
        add %r3, %r4, %r5 # r3 should be 0
        li %r0, 2
        sc
        li %r4, -10
        li %r5, 11
        add %r3, %r5, %r4
        li %r0, 2
        sc
        b l0 l0:
        li %r3, 300
        li %r0, 2
        sc
        li %r6, -100
        li %r7, 100
        add. %r3, %r7, %r6
        bne ex # should not exit
        blt ex
        bgt ex
        bl dispNum
        li %r3, 'G'
        bl dispChar
        beq enterLoop # now it should jump dispNum:
        li %r0, 2
        sc
        blr dispChar:
        li %r0, 0
        sc
        blr enterLoop:
        li %r3, 30
        li %r5, 0
        addi %r5, %r5, -5 loopBody:
        add. %r3, %r3, %r5
        bl dispNum
        bne loopBody
        ba quine-main quine:
        li %r3, 'Q'+0x6100
        bl dispChar
        li %r3, 'u'
        bl dispChar
        li %r3, 'i'
        bl dispChar
        li %r3, 'n'
        bl dispChar
        li %r3, 'e'
        bl dispChar
        li %r1, quine-main
        li %r10, 0
        addi %r1, %r1, -8 quineLoop:
        ldu %r3, 8(%r1)
        add. %r3, %r3, %r10
        bl dispNum
        beq addi_r0
        b quineLoop
        .align 3 end_of_quine:
        .quad 0 addi_r0:
        li %r0, 1
        addi %r3, %r0, -1
        bl dispNum or_sign:
        li %r0, -1
        or. %r0, %r0, %r0
        li %r3, 'a'
        bl dispChar
        bgt ex
        beq ex
        li %r0, 1
        or. %r0, %r0, %r0
        li %r3, 'b'
        bl dispChar
        blt ex
        beq ex
        li %r0, 0
        or. %r0, %r0, %r0
        li %r3, 'c'
        bl dispChar
        bgt ex
        blt ex invalid_ldu:
        li %r3, 'P'
        bl dispChar
        li %r3, 100
        .int 0xe8630009 # ldu %r3, 8(%r3)
        bl dispNum
        li %r3, 100
        li %r0, bignum1-main-8
        .int 0xe8600009 # ldu %r3, 8(%r0)
        bl dispNum mem_read_chk:
        li %r3, 'M'
        bl dispChar
        li %r0, bignum1-main
        ld %r3, 0(%r0)
        bl dispNum
        mr %r3, %r0 # syscall num
        bl dispNum
        li %r1, bignum1-main
        ld %r3, 0(%r1)
        bl dispNum
        b ctr_check ctr_check:
        bdz ex
        bdnz check_so_one_more_time
        li %r3, 'C'
        bl dispChar
        li %r3, 'f'
        bl dispChar
        b ex check_so_one_more_time:
        bso lr_check
        b ex lr_check:
        li %r3, 'L'
        bl dispChar
        li %r3, 50
        li %r31, -1
        li %r0, 2 lr_check_loop:
        add. %r3, %r3, %r31
        sc
        beqlr
        bdnzl lr_check_loop nop_check:
        .quad 0
        .quad 0
        .quad 0
        .quad 0
        .quad 0
        .quad 0
        .quad 0
        .quad 0
        .quad 0
        .quad 0 ascii_check:
        li %r3, 127
        li %r4, -5 ascii_check_loop:
        bl dispChar
        add. %r3, %r3, %r4
        ble success
        b ascii_check_loop success:
        li %r3, 'S'
        bl dispChar
        li %r3, 'u'
        bl dispChar
        li %r3, 'c'
        bl dispChar
        li %r3, 'c'
        bl dispChar
        li %r3, 'e'
        bl dispChar
        li %r3, 's'
        bl dispChar
        bl dispChar
        li %r3, '!'
        bl dispChar
        b ex .align 3 bignum:
        .quad 0x7FFFFFFFFFFFFFFF
        .align 3 bignum_32:
        .quad 0x000000007FFFFFFF smallnum1:
        .quad 12
       .align 3 bignum1:
        .quad 0x7fffffffffffffff end_of_program:
        .quad 0
        .quad 0 ex:
        li %r0, 1
        sc
