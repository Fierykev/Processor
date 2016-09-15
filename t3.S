        .section ".text"
        .global main
        .section ".opd", "aw"
        .global entry
entry:
        .quad main, .TOC.@tocbase,0
        .previous
        .type entry, @function
        .section ".text"
main:
	li %r5, maxnum-main
	ld %r1, 0(%r5)
        li %r2, 0x1
        add %r1, %r1, %r2
        bso exit
        or. %r1, %r1, %r1          # update CR from XER:SO (not set)
        bso exit
        li %r5, maxnum-main
	ld %r1, 0(%r5)
        li %r2, 0x1
        addo. %r1, %r1, %r2        # check if updated SO is used to set CR
        bns exit
        or. %r1, %r1, %r1
        bso invalidCommands        # SO should still be set
	li %r3, 0xBAD
        bl printR3
        b exit
invalidCommands:
	li %r0, 2
	li %r3, 'A'
	.int 0x44000001           # not a valid SC (bit 31 = 1)
	.int 0x44000402		  # LEV != 1 or 0
	.int 0x44000004		  # bit 30 = 0
	li %r3, 'B'
	.int 0x44000002           # valid SC
	li %r3, 8
        .int 0xe8630009           # not a valid load (ra = rt)
	.int 0xe8600009		  # ra = 0
        bl printR3
	bnsl exit                 # SO still set, dont branch, do set LR
	nop#.int 0x00000000           # noop
	nop#.int 0x4e800028           # invalid bclr (xop != 16)
	nop#.int 0x4f800020		  # bo[0]==bo[2] and other bo bits arent all 0
	nop#.int 0x4fe00020		  # bo[0]==bo[2] and other bo bits arent all 0

	# test invalid mtcrf
	li %r3, 'I'
	bsol printR3              # should print 0...049

	li %r5, 0
	# 0111 1100 101(0) (8bit mask) (0)001 0010 000(0)
	# 7CA(FF)120 is proper form

	.int 0x0           # last bit is 1
	bnsl printR3
	addi %r3, %r3, 1
	.int 0x0           # bit 20 is 1
	bnsl printR3
	addi %r3, %r3, 1
	.int 0x7CA00120           # no mask
	bnsl printR3
	addi %r3, %r3, 1
	.int 0x7CBFF120           # bit 11 is 1, more than one field selected
	bnsl printR3
	addi %r3, %r3, 1
	.int 0x7CB80120           # valid command
	bnsl printR3              # print 0...04d

	li %r3, -1
	li %r5, editNumber-main
	.int 0xf8650001           # std %r3, 0(%r5), but last bit is 1 so invalid
	ld %r3, 0(%r5)
	bl printR3                # should print 0

	#test invalid mtspr mfspr
	mfspr %r21, 1
	li %r3, 0
	mtxer %r3
	mtlr %r3
	mtctr %r3
	li %r3, -1
	mfspr %r3, 3              # like mov from XER, but extra bits set
	bl printR3
	.int 0x0          # like mov from LR, but last bit is 1
	bl printR3
	mfspr 0, %r3              # spr is all 0
	bl printR3
	li %r3, -2
	mtspr %r3, 3              # similar
	.int 0x0
	mtspr 0, %r3
	mfspr %r3, 1              # valid
	bl printR3
	mfspr %r3, 8
	bl printR3                # LR is set by branches, should print 0...0124
	mfspr %r3, 9
	bl printR3
	mtspr 1, %r21             # reset spr's
	li %r3, 0
	mtspr 9, %r3

	li %r3, 'w'               # thats a 0...077
	bl printR3
	b pipelineTestStart

	.quad 0xABCDEF0123456789  # should be skipped by branch
printR3:
        li %r0, 2
        sc
        blr
exit:
        li %r0, 1
        sc
	#.int 0                    # memory alignment

	## data ##
maxnum:
        .quad 0x7fffffffffffffff
bigNum:
	.quad 0x0000000000000a55
numOne:
	.quad 0xffffffffffffffff
editNumber:
	.quad 0x0000000000000000
blPrintCommand:
	.quad 0x4bffff4500000000
newXER:
	.quad 0x0000000001000000
thisAddress:
	.quad 0x0000000000000000

pipelineTestStart:
testCommonPipeFault:
	li %r5, bigNum-main
	li %r6, numOne-main
	li %r10, 0
	ld %r1, 0(%r5)
	ld %r2, 0(%r6)
	addo. %r3, %r1, %r2
	bl printR3
	b one
branchingWeaveTest:
one:
	b six
two:
	bso seven
three:
	b eight
four:
	bso nine
five:
	b ten
six:
	bso two
seven:
	b three
eight:
	bso four
nine:
	b five
ten:
	add %r10, %r10, %r1
	addo. %r1, %r1, %r2
	addo. %r1, %r1, %r2
	addo. %r1, %r1, %r2	
	addo. %r1, %r1, %r2
	#bgt branchingWeaveTest
	or %r3, %r10, %r10
	bl printR3
p10Issues:
	li %r5, changeCommand-main
	li %r6, blPrintCommand-main
	ld %r4, 0(%r6)
	std %r4, 0(%r5)
changeCommand:
	b exit                     # this will be changed to a print, prints r3 again (0...a55)
	b exit
	
	# test xer
	li %r5, maxnum-main
	ld %r1, 0(%r5)
        li %r2, 0x1
        addo. %r1, %r1, %r2        # overflow
        bns exit
	li %r15, 0
	mtxer %r15                 # set xer to 0
	bns exit
	or. %r3, %r3, %r3          # update CR
	bso exit
	
	li %r5, maxnum-main
	ld %r1, 0(%r5)
        li %r2, 0x1
        addo. %r1, %r1, %r2
	bns exit
	li %r5, newXER-main
	ld %r5, 0(%r5)
	mtxer %r5
	or. %r3, %r3, %r3
	bso exit
	li %r3, 'V'
	bl printR3

	# none of our instructions should update carry bit
	li %r3, 0
	mtspr 1, %r3
	li %r13, thisAddress-main
	li %r14, -1
	std %r14, 0(%r13)
	ld %r4, 0(%r13)
	ld %r5, 0(%r13)
	or %r3, %r4, %r4
	bl printR3                  # make sure the -1 is stored and loaded
	add %r3, %r4, %r5
	mfspr %r20, 1
	add. %r3, %r4, %r5
	mfspr %r21, 1
	addo %r3, %r4, %r5
	mfspr %r22, 1
	addo. %r3, %r4, %r5
	mfspr %r23, 1
	or %r3, %r20, %r21
	or. %r4, %r22, %r23
	or %r3, %r3, %r4
	bl printR3                  # should print 0, no xer fields are set
	mfspr %r3, 1
	bl printR3                  # value of xer. should be 0

	# branch after updating CR
	li %r4, -1
	li %r5, 0
	li %r6, 1

	li %r3, 1
	add. %r7, %r4, %r5        # less than
	bltl printR3

	addi %r3, %r3, 1
	add. %r7, %r5, %r6        # greater than
	bgtl printR3

	addi %r3, %r3, 1
	add. %r7, %r4, %r6        # equal to
	beql printR3

	# set LR then BCLR
	li %r3, 'a'
	bl printR3
	b skip1                   # LR points here, but is then incremented
	addi %r3, %r3, 1
	bl printR3                # should print 'a' + 10 + 1
	b skip2
skip1:
	addi %r3, %r3, 10
	mfspr %r6, 8
	addi %r6, %r6, 4
	mtspr 8, %r6
	blr
skip2:
	
	# set LR then set new one with branch
	li %r3, 'w'
	li %r8, 0
	mtspr 8, %r8
	bl printR3                # print the w (0...077)
	mfspr %r3, 8
	bl printR3                # not 0, should be address of mfspr %r3, 8 (0...0338)

	# set the CTR then update it with a branch
	li %r3, -1
	li %r8, 1
	mtspr 9, %r8
	bdzl printR3
	mfspr %r3, 9              # counter has been decremented from 1 to 0, print 0
	bl printR3

	# test setting of high order bits upon mfspr from XER
	li %r3, -1
	mtspr 1, %r3
	mfspr %r3, 1              # XER is a 64 bit register in 2.07
	bl printR3
	b exit

