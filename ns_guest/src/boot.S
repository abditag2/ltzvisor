
/* CPU Operation Modes */
#define	USER_MODE		0x10
#define FIQ_MODE		0x11
#define IRQ_MODE		0x12
#define SVC_MODE		0x13
// MON Not Used
#define ABORT_MODE   		0x17
// HYP Not Used
#define UNDEF_MODE		0x1B
#define SYS_MODE		0x1F


/* CPSR bit assignments */
#define CPSR_I_BIT		(1 << 7) // Mask IRQ
#define CPSR_F_BIT		(1 << 6) // Mask IRQ


/* SCTLR bit assignements */
#define	SCTLR_M_BIT		(1 << 0)
#define	SCTLR_A_BIT		(1 << 1)
#define SCTLR_C_BIT		(1 << 2)
#define SCTLR_Z_BIT		(1 << 11)
#define SCTLR_I_BIT		(1 << 12)
#define SCTLR_U_BIT		(1 << 22)
#define SCTLR_XP_BIT		(1 << 23)


.global main
.global ctor_init

.code 32


.section ".startup", "ax"
.global _start
.func _start
_start:

run_cpu0:
	mrc	p15, 0, r0, c0, c0, 5	@ MPIDR
	and	r0, r0, #0x03		@ Mask, leaving only CPU id
	cmp	r0, #0x00		@ CPU0?
	bne	stuck_loop


	/** Set VBAR */
vector_table:
	ldr	r0, =vectors
	mcr	p15, 0, r0, c12, c0, 0


/********************** Init stacks **********************/
init_stacks:

	@ IRQ mode
   	msr	CPSR_c, #(IRQ_MODE | CPSR_I_BIT | CPSR_F_BIT)
    	ldr     sp, =__irq_stack_top

	@ ABORT mode
   	msr	CPSR_c, #(ABORT_MODE | CPSR_I_BIT | CPSR_F_BIT)
    	ldr     sp, =__abort_stack_top

	@ UNDEF mode
   	msr	CPSR_c, #(UNDEF_MODE | CPSR_I_BIT | CPSR_F_BIT)
    	ldr     sp, =__undef_stack_top

	@ SVC 	mode
   	msr	CPSR_c, #(SVC_MODE | CPSR_I_BIT | CPSR_F_BIT)
    	ldr     sp, =__svc_stack_top

	@ USER/SYS mode
   	msr	CPSR_c, #(SYS_MODE | CPSR_F_BIT)
    	ldr     sp, =__sys_stack_top


/****** Init memory (MMU, cache, branch prediction) ******/
init_memory_sys:

	@ MMU
	bl	mmu_disable

	/* MMU now enabled - Virtual Address Space active */

	@ Branch Prediction
	bl	branchPrediction_disable

	@ L1 data Cache
	bl	dCache_disable

	@ L1 instruction Cache
	bl	iCache_disable



/********************** Enable FPU **********************/
enable_fpu:

	/********* TODO *********/


/********************** Clear bss **********************/
zero_bss:
	bl	clear_bss


/************** Disable strict align check *************/
strick_align_check:
	bl	strickAlignCheck_disable


/********************** Call main **********************/
call_main:
	ldr     r12, =main
	blx	r12

	@ Should never return here!
    	b	.

.endfunc	@_start


/********************** Stuck loop **********************/
stuck_loop:
	b	stuck_loop

/**********************************************************************************
 * Clear bss
 * This function is responsible for clearing (zero init) the *.bss section
 **********************************************************************************/
.global clear_bss
.func clear_bss
clear_bss:
	push	{r1-r3}
	ldr	r1,=__bss_start
	ldr     r2,=__bss_end
	mov     r3,#0
1:
	cmp     r1,r2
	stmltia r1!,{r3}
	blt     1b
	pop	{r1-r3}
	bx	lr

.endfunc	@clear_bss

.end
