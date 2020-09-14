	.cpu cortex-m4
	.eabi_attribute 27, 1
	.eabi_attribute 28, 1
	.eabi_attribute 20, 1
	.eabi_attribute 21, 1
	.eabi_attribute 23, 3
	.eabi_attribute 24, 1
	.eabi_attribute 25, 1
	.eabi_attribute 26, 1
	.eabi_attribute 30, 1
	.eabi_attribute 34, 1
	.eabi_attribute 18, 4
	.file	"httpserver_netconn.c"
	.text
	.section	.rodata.http_server_netconn_thread8088.str1.4,"aMS",%progbits,1
	.align	2
.LC0:
	.ascii	"Listening on Port %d...\012\000"
	.align	2
.LC1:
	.ascii	"HTTPD\000"
	.section	.text.http_server_netconn_thread8088,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	http_server_netconn_thread8088, %function
http_server_netconn_thread8088:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}
	sub	sp, sp, #20
	movs	r2, #0
	mov	r1, r2
	movs	r0, #16
	bl	netconn_new_with_proto_and_callback
	cbz	r0, .L1
	mov	r4, r0
	movs	r1, #80
	ldr	r0, .L9
	bl	printf
	movs	r2, #80
	movs	r1, #0
	mov	r0, r4
	bl	netconn_bind
	cbz	r0, .L7
.L1:
	add	sp, sp, #20
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L7:
	movs	r1, #255
	mov	r0, r4
	bl	netconn_listen_with_backlog
	ldr	r7, .L9+4
	ldr	r6, .L9+8
	ldr	r5, .L9+12
	b	.L3
.L8:
	movs	r3, #2
	str	r3, [sp]
	mov	r3, #500
	ldr	r2, [sp, #12]
	mov	r1, r7
	mov	r0, r6
	bl	sys_thread_new
	mov	r3, #268435456
	str	r3, [r5]
	.syntax unified
@ 529 "E:\GIT-Repos\stm32-fw\stm32h7\common_h7\lwip\httpserver_netconn.c" 1
	dsb
@ 0 "" 2
@ 529 "E:\GIT-Repos\stm32-fw\stm32h7\common_h7\lwip\httpserver_netconn.c" 1
	isb
@ 0 "" 2
	.thumb
	.syntax unified
.L3:
	add	r1, sp, #12
	mov	r0, r4
	bl	netconn_accept
	cmp	r0, #0
	bne	.L3
	b	.L8
.L10:
	.align	2
.L9:
	.word	.LC0
	.word	http_server_session
	.word	.LC1
	.word	-536810236
	.size	http_server_netconn_thread8088, .-http_server_netconn_thread8088
	.section	.rodata.HtmlStaticFile.str1.4,"aMS",%progbits,1
	.align	2
.LC2:
	.ascii	"httpd open file %s\012\000"
	.align	2
.LC3:
	.ascii	"httpd failed to open file %s\012\000"
	.section	.text.HtmlStaticFile,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlStaticFile, %function
HtmlStaticFile:
	@ args = 0, pretend = 0, frame = 24
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}
	sub	sp, sp, #36
	mov	r5, r0
	mov	r4, r1
	add	r0, sp, #12
	bl	fs_open
	cbnz	r0, .L12
	mov	r1, r4
	ldr	r0, .L15
	bl	printf
	movs	r3, #0
	str	r3, [sp]
	ldr	r2, [sp, #16]
	ldr	r1, [sp, #12]
	mov	r0, r5
	bl	netconn_write_partly
	add	r0, sp, #12
	bl	fs_close
.L11:
	add	sp, sp, #36
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L12:
	mov	r1, r4
	ldr	r0, .L15+4
	bl	printf
	ldr	r7, .L15+8
	mov	r0, r7
	bl	strlen
	mov	r2, r0
	movs	r6, #0
	str	r6, [sp]
	mov	r3, r6
	mov	r1, r7
	mov	r0, r5
	bl	netconn_write_partly
	mov	r0, r4
	bl	strlen
	mov	r2, r0
	str	r6, [sp]
	movs	r3, #1
	mov	r1, r4
	mov	r0, r5
	bl	netconn_write_partly
	b	.L11
.L16:
	.align	2
.L15:
	.word	.LC2
	.word	.LC3
	.word	.LANCHOR0
	.size	HtmlStaticFile, .-HtmlStaticFile
	.section	.rodata.HtmlPage.str1.4,"aMS",%progbits,1
	.align	2
.LC4:
	.ascii	"%d\000"
	.section	.text.HtmlPage,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlPage, %function
HtmlPage:
	@ args = 0, pretend = 0, frame = 120
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	sub	sp, sp, #132
	mov	r4, r0
	mov	r6, r1
	mov	r8, r2
	ldr	r7, .L32
	mov	r0, r7
	bl	strlen
	mov	r2, r0
	movs	r5, #0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r7
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r3, .L32+4
	lsls	r2, r6, #5
	add	r9, r3, r6, lsl #5
	ldr	r7, [r3, r2]
	mov	r3, r7
	ldr	r2, .L32+8
	movs	r1, #120
	add	r0, sp, #8
	bl	snprintf
	add	r0, sp, #8
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	movs	r3, #1
	add	r1, sp, #8
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r3, [r9, #12]
	movw	r2, #9999
	cmp	r3, r5
	it	eq
	moveq	r3, r2
	ldr	r2, .L32+12
	movs	r1, #120
	add	r0, sp, #8
	bl	snprintf
	add	r0, sp, #8
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	movs	r3, #1
	add	r1, sp, #8
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r9, .L32+36
	mov	r0, r9
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r9
	mov	r0, r4
	bl	netconn_write_partly
	mov	r0, r7
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r7
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r7, .L32+16
	mov	r0, r7
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r7
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r7, .L32+4
	ldr	r10, .L32+40
	mov	r9, r5
	ldr	fp, .L32+44
	b	.L20
.L19:
	adds	r5, r5, #1
	adds	r7, r7, #32
	cmp	r5, #5
	beq	.L31
.L20:
	cmp	r6, r5
	beq	.L19
	mov	r0, r10
	bl	strlen
	mov	r2, r0
	str	r9, [sp]
	mov	r3, r9
	mov	r1, r10
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r3, [r7, #4]
	str	r3, [sp]
	ldr	r3, [r7, #8]
	mov	r2, fp
	movs	r1, #120
	add	r0, sp, #8
	bl	snprintf
	add	r0, sp, #8
	bl	strlen
	mov	r2, r0
	str	r9, [sp]
	movs	r3, #1
	add	r1, sp, #8
	mov	r0, r4
	bl	netconn_write_partly
	b	.L19
.L31:
	ldr	r5, .L32+20
	mov	r0, r5
	bl	strlen
	mov	r2, r0
	movs	r3, #0
	str	r3, [sp]
	mov	r1, r5
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r3, .L32+4
	add	r3, r3, r6, lsl #5
	ldr	r7, [r3, #16]
	cmp	r7, #0
	beq	.L21
	ldr	r9, .L32+48
	mov	r0, r9
	bl	strlen
	mov	r2, r0
	movs	r5, #0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r9
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r9, .L32+32
	mov	r0, r9
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r9
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r3, [r7]
	adds	r3, r3, #1
	str	r3, [r7]
	ldr	r2, .L32+24
	movs	r1, #120
	add	r0, sp, #8
	bl	snprintf
	add	r0, sp, #8
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	movs	r3, #1
	add	r1, sp, #8
	mov	r0, r4
	bl	netconn_write_partly
.L27:
	ldr	r3, .L32+4
	add	r3, r3, r6, lsl #5
	ldrb	r3, [r3, #20]	@ zero_extendqisi2
	cbz	r3, .L22
	cmp	r3, #1
	beq	.L23
	ldr	r5, .L32+28
	mov	r0, r5
	bl	strlen
	mov	r2, r0
	movs	r3, #0
	str	r3, [sp]
	mov	r1, r5
	mov	r0, r4
	bl	netconn_write_partly
	b	.L17
.L22:
	ldr	r1, .L32+4
	add	r6, r1, r6, lsl #5
	ldr	r1, [r6, #24]
	mov	r0, r4
	bl	HtmlStaticFile
.L17:
	add	sp, sp, #132
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L23:
	ldr	r3, [r8, #184]
	cbz	r3, .L26
	ldr	r3, .L32+4
	add	r3, r3, r6, lsl #5
	ldr	r3, [r3, #28]
	cbz	r3, .L26
	mov	r1, r8
	movs	r0, #0
	blx	r3
.L26:
	ldr	r1, .L32+4
	add	r6, r1, r6, lsl #5
	ldr	r3, [r6, #24]
	cmp	r3, #0
	beq	.L17
	movs	r1, #0
	mov	r0, r4
	blx	r3
	b	.L17
.L21:
	ldr	r5, .L32+32
	mov	r0, r5
	bl	strlen
	mov	r2, r0
	movs	r3, #0
	str	r3, [sp]
	mov	r1, r5
	mov	r0, r4
	bl	netconn_write_partly
	b	.L27
.L33:
	.align	2
.L32:
	.word	.LANCHOR1
	.word	.LANCHOR2
	.word	.LANCHOR3
	.word	.LANCHOR4
	.word	.LANCHOR6
	.word	.LANCHOR9
	.word	.LC4
	.word	.LANCHOR12
	.word	.LANCHOR11
	.word	.LANCHOR5
	.word	.LANCHOR7
	.word	.LANCHOR8
	.word	.LANCHOR10
	.size	HtmlPage, .-HtmlPage
	.section	.rodata.http_server_serve.str1.4,"aMS",%progbits,1
	.align	2
.LC5:
	.ascii	"GET \000"
	.align	2
.LC6:
	.ascii	"Params:%s:\012\000"
	.align	2
.LC7:
	.ascii	"Param %d: P=%s, V=%s\012\000"
	.align	2
.LC8:
	.ascii	"HandleGet:%s:\012\000"
	.align	2
.LC9:
	.ascii	"/404.html\000"
	.align	2
.LC10:
	.ascii	"/index.html\000"
	.align	2
.LC11:
	.ascii	"http_server: token too long, truncated\000"
	.section	.text.http_server_serve,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	http_server_serve, %function
http_server_serve:
	@ args = 0, pretend = 0, frame = 304
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	sub	sp, sp, #304
	mov	r4, r0
	add	r1, sp, #300
	bl	netconn_recv
	cbnz	r0, .L35
	ldrsb	r3, [r4, #8]
	cbz	r3, .L70
.L35:
	mov	r0, r4
	bl	netconn_close
	ldr	r0, [sp, #300]
	bl	netbuf_delete
	add	sp, sp, #304
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L70:
	add	r2, sp, #294
	add	r1, sp, #296
	ldr	r0, [sp, #300]
	bl	netbuf_data
	ldrh	r3, [sp, #294]
	cmp	r3, #3
	bls	.L35
	movs	r2, #4
	ldr	r1, .L76
	ldr	r0, [sp, #296]
	bl	strncmp
	cmp	r0, #0
	bne	.L35
	ldrh	r0, [sp, #294]
	subs	r0, r0, #4
	uxth	r0, r0
	strh	r0, [sp, #294]	@ movhi
	ldr	r3, [sp, #296]
	adds	r3, r3, #4
	movs	r5, #0
	add	r1, sp, #212
.L36:
	mov	r6, r3
	cmp	r0, r5
	bls	.L38
	ldrb	r2, [r3], #1	@ zero_extendqisi2
	cmp	r2, #32
	beq	.L38
	cmp	r2, #63
	beq	.L38
	mov	r6, r3
	strb	r2, [r1], #1
	adds	r5, r5, #1
	cmp	r5, #80
	bne	.L36
	movs	r3, #0
	strb	r3, [r1]
	ldr	r0, .L76+4
	bl	puts
	b	.L59
.L51:
	ldrb	r3, [r5, #160]	@ zero_extendqisi2
	ldrb	r2, [r5, #159]	@ zero_extendqisi2
	add	r1, sp, #24
	add	r3, r3, r1
	add	r2, r2, r1
	mov	r1, r7
	mov	r0, r8
	bl	printf
	adds	r7, r7, #1
.L50:
	adds	r5, r5, #2
	cmp	r7, r6
	bne	.L51
	str	r6, [sp, #208]
.L40:
	add	r1, sp, #212
	ldr	r0, .L76+8
	bl	printf
	ldr	r5, .L76+12
	movs	r6, #0
.L53:
	add	r1, sp, #212
	ldr	r0, [r5, #8]
	bl	strcmp
	cbz	r0, .L71
	adds	r6, r6, #1
	adds	r5, r5, #32
	cmp	r6, #5
	bne	.L53
	add	r1, sp, #212
	add	r0, sp, #4
	bl	fs_open
	cbz	r0, .L72
	add	r1, sp, #212
	ldr	r0, .L76+16
	bl	strcmp
	cbz	r0, .L73
	ldr	r1, .L76+20
	mov	r0, r4
	bl	HtmlStaticFile
	b	.L35
.L71:
	add	r2, sp, #24
	mov	r1, r6
	mov	r0, r4
	bl	HtmlPage
	b	.L35
.L72:
	add	r0, sp, #4
	bl	fs_close
	add	r1, sp, #212
	mov	r0, r4
	bl	HtmlStaticFile
	b	.L35
.L73:
	add	r2, sp, #24
	movs	r1, #0
	mov	r0, r4
	bl	HtmlPage
	b	.L35
.L75:
	strb	lr, [r3], #1
	adds	r0, r1, #1
	add	ip, ip, #2
.L57:
	mov	r8, ip
	strb	r0, [ip, #161]
	adds	r2, r3, #1
	ldrb	r1, [r3, #1]	@ zero_extendqisi2
	cbz	r1, .L45
	subs	r0, r0, r3
.L44:
	adds	r7, r2, r0
	cmp	r1, #61
	beq	.L74
	ldrb	r1, [r2, #1]!	@ zero_extendqisi2
	cmp	r1, #0
	bne	.L44
.L45:
	movs	r7, #0
	ldr	r8, .L76+28
	b	.L50
.L74:
	mov	r3, r2
	strb	lr, [r3], #1
	adds	r7, r7, #1
	strb	r7, [r8, #162]
	adds	r6, r6, #1
	ldrb	r2, [r2, #1]	@ zero_extendqisi2
	cmp	r2, #0
	beq	.L45
.L58:
	adds	r1, r3, r0
	cmp	r2, #38
	beq	.L75
	ldrb	r2, [r3, #1]!	@ zero_extendqisi2
	cmp	r2, #0
	bne	.L58
	b	.L45
.L38:
	movs	r3, #0
	strb	r3, [r1]
.L59:
	ldrh	r3, [sp, #294]
	movs	r2, #0
	str	r2, [sp, #208]
	ldrb	r2, [r6]	@ zero_extendqisi2
	cmp	r2, #63
	bne	.L40
	subs	r1, r3, #1
	subs	r1, r1, r5
	uxth	r1, r1
	cmp	r1, #0
	beq	.L40
	movs	r3, #0
	add	r0, sp, #24
.L41:
	cmp	r1, r3
	bls	.L42
	ldrb	r2, [r6, #1]!	@ zero_extendqisi2
	cmp	r2, #32
	beq	.L42
	strb	r2, [r0], #1
	adds	r3, r3, #1
	cmp	r3, #160
	bne	.L41
.L42:
	movs	r7, #0
	strb	r7, [r0]
	add	r1, sp, #24
	ldr	r0, .L76+24
	bl	printf
	add	r5, sp, #24
	mov	ip, r5
	mov	r6, r7
	mov	r0, r7
	mov	r3, r5
	mov	lr, r7
	b	.L57
.L77:
	.align	2
.L76:
	.word	.LC5
	.word	.LC11
	.word	.LC8
	.word	.LANCHOR2
	.word	.LC10
	.word	.LC9
	.word	.LC6
	.word	.LC7
	.size	http_server_serve, .-http_server_serve
	.section	.rodata.http_server_session.str1.4,"aMS",%progbits,1
	.align	2
.LC12:
	.ascii	"Established connection to remote Port %d...\012\000"
	.align	2
.LC13:
	.ascii	"Terminated connection to remote Port %d...\012\000"
	.section	.text.http_server_session,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	http_server_session, %function
http_server_session:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r3, r4, r5, lr}
	mov	r4, r0
	ldr	r3, [r0, #4]
	ldrh	r5, [r3, #24]
	mov	r1, r5
	ldr	r0, .L81
	bl	printf
	mov	r0, r4
	bl	http_server_serve
	mov	r0, r4
	bl	netconn_delete
	mov	r1, r5
	ldr	r0, .L81+4
	bl	printf
	movs	r0, #0
	bl	vTaskDelete
.L79:
	b	.L79
.L82:
	.align	2
.L81:
	.word	.LC12
	.word	.LC13
	.size	http_server_session, .-http_server_session
	.section	.rodata.HtmlOneBoolSetting.str1.4,"aMS",%progbits,1
	.align	2
.LC14:
	.ascii	"\000"
	.align	2
.LC15:
	.ascii	"checked\000"
	.align	2
.LC16:
	.ascii	"1\000"
	.align	2
.LC17:
	.ascii	"0\000"
	.section	.text.HtmlOneBoolSetting,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlOneBoolSetting, %function
HtmlOneBoolSetting:
	@ args = 8, pretend = 0, frame = 160
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}
	sub	sp, sp, #188
	mov	r4, r0
	mov	r5, r2
	mov	r6, r3
	ldr	r7, [sp, #208]
	mov	r3, r1
	ldr	r2, .L89
	movs	r1, #160
	add	r0, sp, #24
	bl	snprintf
	add	r0, sp, #24
	bl	strlen
	mov	r2, r0
	movs	r3, #0
	str	r3, [sp]
	movs	r3, #1
	add	r1, sp, #24
	mov	r0, r4
	bl	netconn_write_partly
	cmp	r6, #1
	beq	.L84
	ldr	r3, .L89+4
	str	r7, [sp, #16]
	str	r3, [sp, #12]
	str	r5, [sp, #8]
	ldr	r2, .L89+8
	str	r2, [sp, #4]
	str	r3, [sp]
	ldr	r2, .L89+12
	movs	r1, #160
	add	r0, sp, #24
	bl	snprintf
	add	r0, sp, #24
	bl	strlen
	mov	r2, r0
	movs	r3, #0
	str	r3, [sp]
	movs	r3, #1
	add	r1, sp, #24
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r3, .L89+16
	ldr	r2, .L89+8
	cmp	r6, #0
	ite	eq
	moveq	r6, r3
	movne	r6, r2
.L85:
	ldr	r3, .L89+20
	ldr	r2, [sp, #212]
	str	r2, [sp, #16]
	str	r3, [sp, #12]
	str	r5, [sp, #8]
	str	r6, [sp, #4]
	str	r3, [sp]
	ldr	r2, .L89+12
	movs	r1, #160
	add	r0, sp, #24
	bl	snprintf
	add	r0, sp, #24
	bl	strlen
	mov	r2, r0
	movs	r5, #0
	str	r5, [sp]
	movs	r3, #1
	add	r1, sp, #24
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r6, .L89+24
	mov	r0, r6
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r6
	mov	r0, r4
	bl	netconn_write_partly
	add	sp, sp, #188
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L84:
	ldr	r3, .L89+4
	str	r7, [sp, #16]
	str	r3, [sp, #12]
	str	r5, [sp, #8]
	ldr	r2, .L89+16
	str	r2, [sp, #4]
	str	r3, [sp]
	ldr	r2, .L89+12
	movs	r1, #160
	add	r0, sp, #24
	bl	snprintf
	add	r0, sp, #24
	bl	strlen
	mov	r2, r0
	movs	r3, #0
	str	r3, [sp]
	movs	r3, #1
	add	r1, sp, #24
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r6, .L89+8
	b	.L85
.L90:
	.align	2
.L89:
	.word	.LANCHOR13
	.word	.LC16
	.word	.LC14
	.word	.LANCHOR14
	.word	.LC15
	.word	.LC17
	.word	.LANCHOR15
	.size	HtmlOneBoolSetting, .-HtmlOneBoolSetting
	.section	.text.HtmlOneDecimalSetting,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlOneDecimalSetting, %function
HtmlOneDecimalSetting:
	@ args = 8, pretend = 0, frame = 160
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, lr}
	sub	sp, sp, #180
	mov	r4, r0
	mov	r5, r2
	mov	r6, r3
	ldr	r8, [sp, #208]
	ldr	r9, [sp, #212]
	str	r9, [sp, #8]
	str	r8, [sp, #4]
	str	r1, [sp]
	mov	r3, r2
	ldr	r2, .L93
	movs	r1, #160
	add	r0, sp, #16
	bl	snprintf
	add	r0, sp, #16
	bl	strlen
	mov	r2, r0
	movs	r7, #0
	str	r7, [sp]
	movs	r3, #1
	add	r1, sp, #16
	mov	r0, r4
	bl	netconn_write_partly
	str	r9, [sp, #8]
	str	r8, [sp, #4]
	str	r5, [sp]
	mov	r3, r6
	ldr	r2, .L93+4
	movs	r1, #160
	add	r0, sp, #16
	bl	snprintf
	add	r0, sp, #16
	bl	strlen
	mov	r2, r0
	str	r7, [sp]
	movs	r3, #1
	add	r1, sp, #16
	mov	r0, r4
	bl	netconn_write_partly
	add	sp, sp, #180
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L94:
	.align	2
.L93:
	.word	.LANCHOR16
	.word	.LANCHOR17
	.size	HtmlOneDecimalSetting, .-HtmlOneDecimalSetting
	.section	.rodata.HtmlOneHexSetting.str1.4,"aMS",%progbits,1
	.align	2
.LC18:
	.ascii	"0x%%0%dx\000"
	.section	.text.HtmlOneHexSetting,"ax",%progbits
	.align	1
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlOneHexSetting, %function
HtmlOneHexSetting:
	@ args = 12, pretend = 0, frame = 176
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, lr}
	sub	sp, sp, #196
	mov	r4, r0
	mov	r6, r1
	mov	r5, r2
	mov	r7, r3
	ldr	r8, [sp, #224]
	mov	r3, r8
	ldr	r2, .L97
	movs	r1, #160
	add	r0, sp, #32
	bl	snprintf
	add	r9, sp, #20
	mov	r3, r7
	add	r2, sp, #32
	movs	r1, #11
	mov	r0, r9
	bl	snprintf
	ldr	r3, [sp, #232]
	str	r3, [sp, #8]
	ldr	r3, [sp, #228]
	str	r3, [sp, #4]
	str	r6, [sp]
	mov	r3, r5
	ldr	r2, .L97+4
	movs	r1, #160
	add	r0, sp, #32
	bl	snprintf
	add	r0, sp, #32
	bl	strlen
	mov	r2, r0
	movs	r6, #0
	str	r6, [sp]
	movs	r3, #1
	add	r1, sp, #32
	mov	r0, r4
	bl	netconn_write_partly
	str	r5, [sp, #4]
	str	r9, [sp]
	mov	r3, r8
	ldr	r2, .L97+8
	movs	r1, #160
	add	r0, sp, #32
	bl	snprintf
	add	r0, sp, #32
	bl	strlen
	mov	r2, r0
	str	r6, [sp]
	movs	r3, #1
	add	r1, sp, #32
	mov	r0, r4
	bl	netconn_write_partly
	add	sp, sp, #196
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L98:
	.align	2
.L97:
	.word	.LC18
	.word	.LANCHOR18
	.word	.LANCHOR19
	.size	HtmlOneHexSetting, .-HtmlOneHexSetting
	.section	.rodata.HtmlTaskList.str1.4,"aMS",%progbits,1
	.align	2
.LC20:
	.ascii	"<pre>\000"
	.align	2
.LC19:
	.ascii	"<br>\000"
	.section	.text.HtmlTaskList,"ax",%progbits
	.align	1
	.global	HtmlTaskList
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlTaskList, %function
HtmlTaskList:
	@ args = 0, pretend = 0, frame = 128
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, lr}
	sub	sp, sp, #140
	mov	r5, r0
	ldr	r3, .L107
	ldm	r3, {r0, r1}
	str	r0, [sp, #8]
	strb	r1, [sp, #12]
	movs	r4, #0
	str	r4, [sp]
	movs	r3, #1
	movs	r2, #5
	ldr	r1, .L107+4
	mov	r0, r5
	bl	netconn_write_partly
	movs	r1, #1
	mov	r0, r4
	bl	TaskSetListStartStop
	movs	r6, #80
.L100:
	add	r3, sp, #8
	mov	r2, r6
	add	r1, sp, #16
	mov	r0, r4
	bl	TaskIterateList
	subs	r4, r0, #0
	blt	.L106
	add	r0, sp, #16
	bl	strlen
	mov	r2, r0
	movs	r3, #0
	str	r3, [sp]
	movs	r3, #1
	add	r1, sp, #16
	mov	r0, r5
	bl	netconn_write_partly
	b	.L100
.L106:
	add	r1, sp, #8
	movs	r0, #1
	bl	MSGD_GetTasklistLine
	bl	MSGD_WaitForTasklistLine
	mov	r4, r0
	cbz	r0, .L99
	movs	r6, #0
	movs	r7, #1
.L103:
	mov	r0, r4
	bl	strlen
	mov	r2, r0
	str	r6, [sp]
	mov	r3, r7
	mov	r1, r4
	mov	r0, r5
	bl	netconn_write_partly
	add	r1, sp, #8
	mov	r0, r6
	bl	MSGD_GetTasklistLine
	bl	MSGD_WaitForTasklistLine
	mov	r4, r0
	cmp	r0, #0
	bne	.L103
.L99:
	add	sp, sp, #140
	@ sp needed
	pop	{r4, r5, r6, r7, pc}
.L108:
	.align	2
.L107:
	.word	.LC19
	.word	.LC20
	.size	HtmlTaskList, .-HtmlTaskList
	.section	.text.HtmlShowEthIf,"ax",%progbits
	.align	1
	.global	HtmlShowEthIf
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlShowEthIf, %function
HtmlShowEthIf:
	@ args = 0, pretend = 0, frame = 120
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	sub	sp, sp, #128
	mov	r5, r0
	ldr	r6, .L116
	mov	r0, r6
	bl	strlen
	mov	r2, r0
	movs	r4, #0
	str	r4, [sp]
	mov	r3, r4
	mov	r1, r6
	mov	r0, r5
	bl	netconn_write_partly
	str	r4, [sp]
	movs	r3, #1
	movs	r2, #5
	ldr	r1, .L116+4
	mov	r0, r5
	bl	netconn_write_partly
	bl	ETHSTAT_GetLineCount
	cbz	r0, .L109
	mov	r6, r0
	movs	r7, #120
	mov	r8, r4
	b	.L112
.L115:
	add	r0, sp, #8
	bl	strlen
	mov	r2, r0
	str	r8, [sp]
	movs	r3, #1
	add	r1, sp, #8
	mov	r0, r5
	bl	netconn_write_partly
.L111:
	adds	r4, r4, #1
	cmp	r6, r4
	beq	.L109
.L112:
	mov	r2, r4
	mov	r1, r7
	add	r0, sp, #8
	bl	ETHSTAT_GetLine
	cmp	r0, #0
	bne	.L115
	b	.L111
.L109:
	add	sp, sp, #128
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L117:
	.align	2
.L116:
	.word	.LANCHOR20
	.word	.LC20
	.size	HtmlShowEthIf, .-HtmlShowEthIf
	.section	.rodata.http_server_netconn_init.str1.4,"aMS",%progbits,1
	.align	2
.LC21:
	.ascii	"HTTP8088\000"
	.section	.text.http_server_netconn_init,"ax",%progbits
	.align	1
	.global	http_server_netconn_init
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	http_server_netconn_init, %function
http_server_netconn_init:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{lr}
	sub	sp, sp, #12
	movs	r3, #1
	str	r3, [sp]
	mov	r3, #500
	movs	r2, #0
	ldr	r1, .L120
	ldr	r0, .L120+4
	bl	sys_thread_new
	add	sp, sp, #12
	@ sp needed
	ldr	pc, [sp], #4
.L121:
	.align	2
.L120:
	.word	http_server_netconn_thread8088
	.word	.LC21
	.size	http_server_netconn_init, .-http_server_netconn_init
	.section	.rodata.HtmlOneSetting.str1.4,"aMS",%progbits,1
	.align	2
.LC22:
	.ascii	"V%d\000"
	.align	2
.LC23:
	.ascii	"Off\000"
	.align	2
.LC24:
	.ascii	"On\000"
	.align	2
.LC25:
	.ascii	"No\000"
	.align	2
.LC26:
	.ascii	"Yes\000"
	.align	2
.LC27:
	.ascii	"HTML rendering for setting type %d not implemented\012"
	.ascii	"\000"
	.section	.text.HtmlOneSetting,"ax",%progbits
	.align	1
	.global	HtmlOneSetting
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlOneSetting, %function
HtmlOneSetting:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, lr}
	sub	sp, sp, #28
	mov	r5, r0
	mov	r4, r1
	ldrb	r3, [r1, #2]	@ zero_extendqisi2
	ldr	r2, .L132
	movs	r1, #6
	add	r0, sp, #16
	bl	snprintf
	ldrb	r1, [r4, #6]	@ zero_extendqisi2
	cmp	r1, #5
	bhi	.L123
	tbb	[pc, r1]
.L125:
	.byte	(.L129-.L125)/2
	.byte	(.L128-.L125)/2
	.byte	(.L127-.L125)/2
	.byte	(.L126-.L125)/2
	.byte	(.L123-.L125)/2
	.byte	(.L124-.L125)/2
	.p2align 1
.L129:
	ldr	r3, .L132+4
	str	r3, [sp, #4]
	ldr	r3, .L132+8
	str	r3, [sp]
	ldrb	r3, [r4, #3]	@ zero_extendqisi2
	add	r2, sp, #16
	ldr	r1, [r4, #8]
	mov	r0, r5
	bl	HtmlOneBoolSetting
.L122:
	add	sp, sp, #28
	@ sp needed
	pop	{r4, r5, pc}
.L128:
	ldr	r3, .L132+12
	str	r3, [sp, #4]
	ldr	r3, .L132+16
	str	r3, [sp]
	ldrb	r3, [r4, #3]	@ zero_extendqisi2
	add	r2, sp, #16
	ldr	r1, [r4, #8]
	mov	r0, r5
	bl	HtmlOneBoolSetting
	b	.L122
.L127:
	ldrb	r3, [r4, #5]	@ zero_extendqisi2
	str	r3, [sp, #4]
	ldrb	r3, [r4, #4]	@ zero_extendqisi2
	str	r3, [sp]
	ldrb	r3, [r4, #3]	@ zero_extendqisi2
	add	r2, sp, #16
	ldr	r1, [r4, #8]
	mov	r0, r5
	bl	HtmlOneDecimalSetting
	b	.L122
.L126:
	ldrb	r3, [r4, #5]	@ zero_extendqisi2
	str	r3, [sp, #8]
	ldrb	r3, [r4, #4]	@ zero_extendqisi2
	str	r3, [sp, #4]
	movs	r3, #2
	str	r3, [sp]
	ldrb	r3, [r4, #3]	@ zero_extendqisi2
	add	r2, sp, #16
	ldr	r1, [r4, #8]
	mov	r0, r5
	bl	HtmlOneHexSetting
	b	.L122
.L124:
	ldrb	r3, [r4, #5]	@ zero_extendqisi2
	str	r3, [sp, #8]
	ldrb	r3, [r4, #4]	@ zero_extendqisi2
	str	r3, [sp, #4]
	movs	r3, #4
	str	r3, [sp]
	ldrb	r3, [r4, #3]	@ zero_extendqisi2
	add	r2, sp, #16
	ldr	r1, [r4, #8]
	mov	r0, r5
	bl	HtmlOneHexSetting
	b	.L122
.L123:
	ldr	r0, .L132+20
	bl	printf
	b	.L122
.L133:
	.align	2
.L132:
	.word	.LC22
	.word	.LC23
	.word	.LC24
	.word	.LC25
	.word	.LC26
	.word	.LC27
	.size	HtmlOneSetting, .-HtmlOneSetting
	.section	.rodata.HtmlSettingsCM7.str1.4,"aMS",%progbits,1
	.align	2
.LC28:
	.ascii	"/js/only_chgd.js\000"
	.section	.text.HtmlSettingsCM7,"ax",%progbits
	.align	1
	.global	HtmlSettingsCM7
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlSettingsCM7, %function
HtmlSettingsCM7:
	@ args = 0, pretend = 0, frame = 0
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	sub	sp, sp, #8
	mov	r4, r0
	ldr	r6, .L139
	mov	r0, r6
	bl	strlen
	mov	r2, r0
	movs	r5, #0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r6
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r6, .L139+4
	mov	r0, r6
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r6
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r1, .L139+8
	mov	r0, r4
	bl	HtmlStaticFile
	movs	r0, #1
	bl	MSGD_GetSettingsLine
	bl	MSGD_WaitForGetSettingsLine
	mov	r1, r0
	ldrb	r3, [r0]	@ zero_extendqisi2
	cbz	r3, .L135
.L136:
	mov	r0, r4
	bl	HtmlOneSetting
	mov	r0, r5
	bl	MSGD_GetSettingsLine
	bl	MSGD_WaitForGetSettingsLine
	mov	r1, r0
	ldrb	r3, [r0]	@ zero_extendqisi2
	cmp	r3, #0
	bne	.L136
.L135:
	ldr	r5, .L139+12
	mov	r0, r5
	bl	strlen
	mov	r2, r0
	movs	r3, #0
	str	r3, [sp]
	mov	r1, r5
	mov	r0, r4
	bl	netconn_write_partly
	add	sp, sp, #8
	@ sp needed
	pop	{r4, r5, r6, pc}
.L140:
	.align	2
.L139:
	.word	.LANCHOR21
	.word	.LANCHOR22
	.word	.LC28
	.word	.LANCHOR23
	.size	HtmlSettingsCM7, .-HtmlSettingsCM7
	.section	.text.HtmlSettingsCM4,"ax",%progbits
	.align	1
	.global	HtmlSettingsCM4
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlSettingsCM4, %function
HtmlSettingsCM4:
	@ args = 0, pretend = 0, frame = 16
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, lr}
	sub	sp, sp, #24
	mov	r6, r0
	ldr	r5, .L146
	mov	r0, r5
	bl	strlen
	mov	r2, r0
	movs	r4, #0
	str	r4, [sp]
	mov	r3, r4
	mov	r1, r5
	mov	r0, r6
	bl	netconn_write_partly
	ldr	r5, .L146+4
	mov	r0, r5
	bl	strlen
	mov	r2, r0
	str	r4, [sp]
	mov	r3, r4
	mov	r1, r5
	mov	r0, r6
	bl	netconn_write_partly
	ldr	r1, .L146+8
	mov	r0, r6
	bl	HtmlStaticFile
	bl	Config_GetCnt
	cbz	r0, .L142
	mov	r7, r0
	ldr	r4, .L146+12
	movs	r5, #0
	mov	r8, #1
.L143:
	strb	r8, [sp, #12]
	uxtb	r0, r5
	strb	r0, [sp, #14]
	ldr	r3, [r4, #4]
	str	r3, [sp, #20]
	ldrb	r3, [r4, #2]	@ zero_extendqisi2
	strb	r3, [sp, #17]
	ldrb	r3, [r4, #1]	@ zero_extendqisi2
	strb	r3, [sp, #16]
	ldrb	r3, [r4, #3]	@ zero_extendqisi2
	strb	r3, [sp, #18]
	bl	Config_GetVal
	strb	r0, [sp, #15]
	add	r1, sp, #12
	mov	r0, r6
	bl	HtmlOneSetting
	adds	r5, r5, #1
	adds	r4, r4, #8
	cmp	r7, r5
	bne	.L143
.L142:
	ldr	r4, .L146+16
	mov	r0, r4
	bl	strlen
	mov	r2, r0
	movs	r3, #0
	str	r3, [sp]
	mov	r1, r4
	mov	r0, r6
	bl	netconn_write_partly
	add	sp, sp, #24
	@ sp needed
	pop	{r4, r5, r6, r7, r8, pc}
.L147:
	.align	2
.L146:
	.word	.LANCHOR24
	.word	.LANCHOR22
	.word	.LC28
	.word	eelimits
	.word	.LANCHOR23
	.size	HtmlSettingsCM4, .-HtmlSettingsCM4
	.section	.rodata.HtmlSettingsTEST.str1.4,"aMS",%progbits,1
	.align	2
.LC29:
	.ascii	"vn\000"
	.align	2
.LC30:
	.ascii	"Rainer\000"
	.align	2
.LC31:
	.ascii	"Vorname\000"
	.align	2
.LC32:
	.ascii	"nn\000"
	.align	2
.LC33:
	.ascii	"Name\000"
	.align	2
.LC34:
	.ascii	"xn\000"
	.align	2
.LC35:
	.ascii	"Keks\000"
	.align	2
.LC36:
	.ascii	"dn\000"
	.align	2
.LC37:
	.ascii	"Decval\000"
	.align	2
.LC38:
	.ascii	"dx\000"
	.align	2
.LC39:
	.ascii	"Hexval2\000"
	.align	2
.LC40:
	.ascii	"dy\000"
	.align	2
.LC41:
	.ascii	"Hexval4\000"
	.align	2
.LC42:
	.ascii	"dz\000"
	.align	2
.LC43:
	.ascii	"Hexval8\000"
	.section	.text.HtmlSettingsTEST,"ax",%progbits
	.align	1
	.global	HtmlSettingsTEST
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlSettingsTEST, %function
HtmlSettingsTEST:
	@ args = 0, pretend = 0, frame = 160
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	sub	sp, sp, #176
	mov	r4, r0
	ldr	r6, .L150
	mov	r0, r6
	bl	strlen
	mov	r2, r0
	movs	r5, #0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r6
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r6, .L150+4
	mov	r0, r6
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r6
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r1, .L150+8
	mov	r0, r4
	bl	HtmlStaticFile
	ldr	r3, .L150+12
	str	r3, [sp, #8]
	ldr	r2, .L150+16
	str	r2, [sp, #4]
	ldr	r2, .L150+20
	str	r2, [sp]
	ldr	r2, .L150+24
	movs	r1, #160
	add	r0, sp, #16
	bl	snprintf
	add	r0, sp, #16
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	movs	r3, #1
	add	r1, sp, #16
	mov	r0, r4
	bl	netconn_write_partly
	ldr	r3, .L150+28
	str	r3, [sp, #4]
	ldr	r3, .L150+32
	str	r3, [sp]
	movs	r3, #1
	ldr	r2, .L150+36
	ldr	r1, .L150+40
	mov	r0, r4
	bl	HtmlOneBoolSetting
	ldr	r3, .L150+44
	str	r3, [sp, #4]
	ldr	r3, .L150+48
	str	r3, [sp]
	mov	r3, r5
	ldr	r2, .L150+52
	ldr	r1, .L150+56
	mov	r0, r4
	bl	HtmlOneBoolSetting
	movs	r6, #255
	str	r6, [sp, #4]
	str	r5, [sp]
	movs	r3, #24
	ldr	r2, .L150+60
	ldr	r1, .L150+64
	mov	r0, r4
	bl	HtmlOneDecimalSetting
	str	r6, [sp, #8]
	str	r5, [sp, #4]
	movs	r3, #2
	str	r3, [sp]
	movs	r3, #171
	ldr	r2, .L150+68
	ldr	r1, .L150+72
	mov	r0, r4
	bl	HtmlOneHexSetting
	movw	r3, #65535
	str	r3, [sp, #8]
	str	r5, [sp, #4]
	movs	r3, #4
	str	r3, [sp]
	movw	r3, #43981
	ldr	r2, .L150+76
	ldr	r1, .L150+80
	mov	r0, r4
	bl	HtmlOneHexSetting
	mov	r3, #-1
	str	r3, [sp, #8]
	str	r5, [sp, #4]
	movs	r3, #8
	str	r3, [sp]
	ldr	r3, .L150+84
	ldr	r2, .L150+88
	ldr	r1, .L150+92
	mov	r0, r4
	bl	HtmlOneHexSetting
	ldr	r6, .L150+96
	mov	r0, r6
	bl	strlen
	mov	r2, r0
	str	r5, [sp]
	mov	r3, r5
	mov	r1, r6
	mov	r0, r4
	bl	netconn_write_partly
	add	sp, sp, #176
	@ sp needed
	pop	{r4, r5, r6, pc}
.L151:
	.align	2
.L150:
	.word	.LANCHOR21
	.word	.LANCHOR22
	.word	.LC28
	.word	.LC29
	.word	.LC30
	.word	.LC31
	.word	.LANCHOR25
	.word	.LC25
	.word	.LC26
	.word	.LC32
	.word	.LC33
	.word	.LC23
	.word	.LC24
	.word	.LC34
	.word	.LC35
	.word	.LC36
	.word	.LC37
	.word	.LC38
	.word	.LC39
	.word	.LC40
	.word	.LC41
	.word	-559038737
	.word	.LC42
	.word	.LC43
	.word	.LANCHOR23
	.size	HtmlSettingsTEST, .-HtmlSettingsTEST
	.section	.rodata.GetTagIndex.str1.4,"aMS",%progbits,1
	.align	2
.LC44:
	.ascii	"V\000"
	.section	.text.GetTagIndex,"ax",%progbits
	.align	1
	.global	GetTagIndex
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	GetTagIndex, %function
GetTagIndex:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, lr}
	sub	sp, sp, #8
	mov	r5, r0
	ldr	r6, .L156
	mov	r0, r6
	bl	strlen
	mov	r4, r0
	mov	r2, r0
	mov	r1, r5
	mov	r0, r6
	bl	strncmp
	cbnz	r0, .L154
	movs	r2, #10
	add	r1, sp, #4
	adds	r0, r5, r4
	bl	strtol
	ldr	r3, [sp, #4]
	ldrb	r3, [r3]	@ zero_extendqisi2
	cmp	r3, #0
	it	ne
	movne	r0, #-1
.L152:
	add	sp, sp, #8
	@ sp needed
	pop	{r4, r5, r6, pc}
.L154:
	mov	r0, #-1
	b	.L152
.L157:
	.align	2
.L156:
	.word	.LC44
	.size	GetTagIndex, .-GetTagIndex
	.section	.rodata.GetTagValue.str1.4,"aMS",%progbits,1
	.align	2
.LC45:
	.ascii	"HTML extract value for type %d not implemented\012\000"
	.section	.text.GetTagValue,"ax",%progbits
	.align	1
	.global	GetTagValue
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	GetTagValue, %function
GetTagValue:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, lr}
	sub	sp, sp, #8
	mov	r4, r2
	cmp	r1, #7
	bhi	.L159
	tbb	[pc, r1]
.L161:
	.byte	(.L162-.L161)/2
	.byte	(.L162-.L161)/2
	.byte	(.L162-.L161)/2
	.byte	(.L160-.L161)/2
	.byte	(.L162-.L161)/2
	.byte	(.L160-.L161)/2
	.byte	(.L162-.L161)/2
	.byte	(.L160-.L161)/2
	.p2align 1
.L162:
	movs	r2, #10
	add	r1, sp, #4
	bl	strtol
	str	r0, [r4]
	ldr	r3, [sp, #4]
	ldrb	r0, [r3]	@ zero_extendqisi2
	subs	r0, r0, #0
	it	ne
	movne	r0, #1
	rsbs	r0, r0, #0
.L158:
	add	sp, sp, #8
	@ sp needed
	pop	{r4, pc}
.L160:
	ldrb	r3, [r0]	@ zero_extendqisi2
	cmp	r3, #48
	beq	.L166
.L164:
	movs	r2, #16
	add	r1, sp, #4
	bl	strtol
	str	r0, [r4]
	ldr	r3, [sp, #4]
	ldrb	r0, [r3]	@ zero_extendqisi2
	subs	r0, r0, #0
	it	ne
	movne	r0, #1
	rsbs	r0, r0, #0
	b	.L158
.L166:
	ldrb	r3, [r0, #1]	@ zero_extendqisi2
	and	r3, r3, #223
	cmp	r3, #88
	it	eq
	addeq	r0, r0, #2
	b	.L164
.L159:
	ldr	r0, .L167
	bl	printf
	movs	r0, #0
	b	.L158
.L168:
	.align	2
.L167:
	.word	.LC45
	.size	GetTagValue, .-GetTagValue
	.section	.rodata.HtmlSetCM7.str1.4,"aMS",%progbits,1
	.align	2
.LC46:
	.ascii	"SetCM7: Index %d out of bounds\012\000"
	.align	2
.LC47:
	.ascii	"SetCM7: Failede to set config[%d] to %d\012\000"
	.section	.text.HtmlSetCM7,"ax",%progbits
	.align	1
	.global	HtmlSetCM7
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlSetCM7, %function
HtmlSetCM7:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
	sub	sp, sp, #12
	mov	r7, r1
	movs	r0, #1
	bl	MSGD_GetSettingsLine
	bl	MSGD_WaitForGetSettingsLine
	ldrb	r9, [r0, #1]	@ zero_extendqisi2
	ldr	r3, [r7, #184]
	cbz	r3, .L169
	mov	r5, r7
	movs	r6, #0
	ldr	r10, .L177+4
	ldr	fp, .L177+8
	b	.L173
.L176:
	mov	r1, r0
	mov	r0, fp
	bl	printf
.L172:
	adds	r6, r6, #1
	adds	r5, r5, #2
	ldr	r3, [r7, #184]
	cmp	r3, r6
	bls	.L169
.L173:
	ldrb	r0, [r5, #161]	@ zero_extendqisi2
	add	r0, r0, r7
	bl	GetTagIndex
	mov	r4, r0
	cmp	r0, r9
	bhi	.L176
	cmp	r0, #0
	blt	.L172
	add	r3, r10, r0, lsl #3
	ldrb	r0, [r5, #162]	@ zero_extendqisi2
	add	r2, sp, #4
	ldrb	r1, [r3, #3]	@ zero_extendqisi2
	add	r0, r0, r7
	bl	GetTagValue
	cmp	r0, #0
	blt	.L172
	ldrb	r1, [sp, #4]	@ zero_extendqisi2
	uxtb	r0, r4
	bl	MSGD_SetSettingsLine
	bl	MSGD_WaitForSetSettingsLine
	cmp	r0, #0
	bne	.L172
	ldr	r2, [sp, #4]
	mov	r1, r4
	ldr	r0, .L177
	bl	printf
	b	.L172
.L169:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, r10, fp, pc}
.L178:
	.align	2
.L177:
	.word	.LC47
	.word	eelimits
	.word	.LC46
	.size	HtmlSetCM7, .-HtmlSetCM7
	.section	.rodata.HtmlSetCM4.str1.4,"aMS",%progbits,1
	.align	2
.LC48:
	.ascii	"SetCM4: Index %d out of bounds\012\000"
	.align	2
.LC49:
	.ascii	"SetCM4: Failed to set config[%d] to %d\012\000"
	.section	.text.HtmlSetCM4,"ax",%progbits
	.align	1
	.global	HtmlSetCM4
	.syntax unified
	.thumb
	.thumb_func
	.fpu fpv5-sp-d16
	.type	HtmlSetCM4, %function
HtmlSetCM4:
	@ args = 0, pretend = 0, frame = 8
	@ frame_needed = 0, uses_anonymous_args = 0
	ldr	r3, [r1, #184]
	cmp	r3, #0
	beq	.L186
	push	{r4, r5, r6, r7, r8, r9, lr}
	sub	sp, sp, #12
	mov	r6, r1
	movs	r5, #0
	ldr	r7, .L191
	ldr	r9, .L191+4
	ldr	r8, .L191+8
	b	.L183
.L190:
	mov	r1, r4
	mov	r0, r8
	bl	printf
.L182:
	adds	r5, r5, #1
	ldr	r3, [r6, #184]
	cmp	r3, r5
	bls	.L189
.L183:
	add	r3, r6, #161
	ldrb	r0, [r3, r5, lsl #1]	@ zero_extendqisi2
	add	r0, r0, r6
	bl	GetTagIndex
	mov	r4, r0
	bl	Config_GetCnt
	cmp	r0, r4
	bcc	.L190
	cmp	r4, #0
	blt	.L182
	add	r3, r7, r4, lsl #3
	add	r2, r6, #162
	ldrb	r0, [r2, r5, lsl #1]	@ zero_extendqisi2
	add	r2, sp, #4
	ldrb	r1, [r3, #3]	@ zero_extendqisi2
	add	r0, r0, r6
	bl	GetTagValue
	cmp	r0, #0
	blt	.L182
	ldrb	r1, [sp, #4]	@ zero_extendqisi2
	uxtb	r0, r4
	bl	Config_SetVal
	cmp	r0, #0
	bne	.L182
	ldr	r2, [sp, #4]
	mov	r1, r4
	mov	r0, r9
	bl	printf
	b	.L182
.L189:
	add	sp, sp, #12
	@ sp needed
	pop	{r4, r5, r6, r7, r8, r9, pc}
.L186:
	bx	lr
.L192:
	.align	2
.L191:
	.word	eelimits
	.word	.LC49
	.word	.LC48
	.size	HtmlSetCM4, .-HtmlSetCM4
	.global	form_postfix
	.global	form_prefix
	.global	ethif_header
	.global	cm4_header
	.global	cm7_header
	.section	.rodata.str1.4,"aMS",%progbits,1
	.align	2
.LC50:
	.ascii	"STM32H745 WebServer\000"
	.align	2
.LC51:
	.ascii	"Startseite\000"
	.align	2
.LC52:
	.ascii	"/\000"
	.align	2
.LC53:
	.ascii	"/home.html\000"
	.align	2
.LC54:
	.ascii	"Task List\000"
	.align	2
.LC55:
	.ascii	"Tasklist\000"
	.align	2
.LC56:
	.ascii	"/tasks.html\000"
	.align	2
.LC57:
	.ascii	"Settings CM7\000"
	.align	2
.LC58:
	.ascii	"/settings_cm7.html\000"
	.align	2
.LC59:
	.ascii	"Settings CM4\000"
	.align	2
.LC60:
	.ascii	"/settings_cm4.html\000"
	.align	2
.LC61:
	.ascii	"ETH IF Statistic\000"
	.align	2
.LC62:
	.ascii	"ETH Interface\000"
	.align	2
.LC63:
	.ascii	"/eth_if.html\000"
	.global	TaskListPageHits
	.section	.bss.TaskListPageHits,"aw",%nobits
	.align	2
	.type	TaskListPageHits, %object
	.size	TaskListPageHits, 4
TaskListPageHits:
	.space	4
	.section	.data.bool_setting_rowyn,"aw"
	.align	2
	.set	.LANCHOR14,. + 0
	.type	bool_setting_rowyn, %object
	.size	bool_setting_rowyn, 98
bool_setting_rowyn:
	.ascii	"<input type=\"radio\" value=%s id=\"%s\" %s name=\""
	.ascii	"%s\" onchange=\"add(this)\"><label for=\"%s\">%s</l"
	.ascii	"abel>\000"
	.section	.data.decimal_setting_lbl,"aw"
	.align	2
	.set	.LANCHOR16,. + 0
	.type	decimal_setting_lbl, %object
	.size	decimal_setting_lbl, 50
decimal_setting_lbl:
	.ascii	"<tr><td><label for=\"%s\">%s(%d..%d) :</label></td>"
	.ascii	"\000"
	.section	.data.decimal_setting_val,"aw"
	.align	2
	.set	.LANCHOR17,. + 0
	.type	decimal_setting_val, %object
	.size	decimal_setting_val, 92
decimal_setting_val:
	.ascii	"<td><input type=\"number\" value =%d name=\"%s\" mi"
	.ascii	"n=%d max=%d onchange=\"add(this)\"></td></tr>\012\000"
	.section	.data.hex_setting_lbl,"aw"
	.align	2
	.set	.LANCHOR18,. + 0
	.type	hex_setting_lbl, %object
	.size	hex_setting_lbl, 50
hex_setting_lbl:
	.ascii	"<tr><td><label for=\"%s\">%s(%X..%X) :</label></td>"
	.ascii	"\000"
	.section	.data.hex_setting_val,"aw"
	.align	2
	.set	.LANCHOR19,. + 0
	.type	hex_setting_val, %object
	.size	hex_setting_val, 96
hex_setting_val:
	.ascii	"<td><input pattern=\"0x[A-Fa-f0-9]{1,%d}\" value =\""
	.ascii	"%s\" name=\"%s\" onchange=\"add(this)\"></td></tr>\012"
	.ascii	"\000"
	.section	.data.row_postfix,"aw"
	.align	2
	.set	.LANCHOR15,. + 0
	.type	row_postfix, %object
	.size	row_postfix, 20
row_postfix:
	.ascii	"</td></label></tr>\012\000"
	.section	.data.row_prefix,"aw"
	.align	2
	.set	.LANCHOR13,. + 0
	.type	row_prefix, %object
	.size	row_prefix, 28
row_prefix:
	.ascii	"<tr><td><label>%s:</td><td>\000"
	.section	.data.text_setting_row,"aw"
	.align	2
	.set	.LANCHOR25,. + 0
	.type	text_setting_row, %object
	.size	text_setting_row, 119
text_setting_row:
	.ascii	" <tr><td><label for=\"%s\">%s:</label></td><td><inp"
	.ascii	"ut type=\"text\" value =\"%s\" name=\"%s\" onchange"
	.ascii	"=\"add(this)\"></td></tr>\012\000"
	.section	.rodata.BODY_POSTFIX,"a"
	.align	2
	.set	.LANCHOR11,. + 0
	.type	BODY_POSTFIX, %object
	.size	BODY_POSTFIX, 30
BODY_POSTFIX:
	.ascii	"</span></small></body></html>\000"
	.section	.rodata.COL_PREFIX,"a"
	.align	2
	.set	.LANCHOR7,. + 0
	.type	COL_PREFIX, %object
	.size	COL_PREFIX, 127
COL_PREFIX:
	.ascii	"<td style=\"font-family:Verdana;font-weight:bold;fo"
	.ascii	"nt-style:italic;background-color:rgb(51, 51, 255);t"
	.ascii	"ext-align:center\"><small>\000"
	.section	.rodata.COL_s_s,"a"
	.align	2
	.set	.LANCHOR8,. + 0
	.type	COL_s_s, %object
	.size	COL_s_s, 66
COL_s_s:
	.ascii	"<a href=\"%s\"><span style=\"color:white\">%s</span"
	.ascii	"></a></small></td>\000"
	.section	.rodata.FileErr.13725,"a"
	.align	2
	.set	.LANCHOR0,. + 0
	.type	FileErr.13725, %object
	.size	FileErr.13725, 17
FileErr.13725:
	.ascii	"File not found: \000"
	.section	.rodata.HEADER,"a"
	.align	2
	.set	.LANCHOR1,. + 0
	.type	HEADER, %object
	.size	HEADER, 164
HEADER:
	.ascii	"<!DOCTYPE html PUBLIC \"-//W3C//DTD HTML 4.01//EN\""
	.ascii	" \"http://www.w3.org/TR/html4/strict.dtd\"><meta ht"
	.ascii	"tp-equiv=\"Content-Type\" content=\"text/html;chars"
	.ascii	"et=windows-1252\">\000"
	.section	.rodata.HEADER2,"a"
	.align	2
	.set	.LANCHOR5,. + 0
	.type	HEADER2, %object
	.size	HEADER2, 353
HEADER2:
	.ascii	"<style =\"font-weight:normal;font-family:Verdana\">"
	.ascii	"</style></head><body><table style=\"width:961px\"><"
	.ascii	"tr style=\"height:100px\"><td><a href=\"http://mega"
	.ascii	"experte.de\"><img style=\"width: 100px; height: 100"
	.ascii	"px;\" alt=\"Idiot\" src=\"im/idiot.png\"> </a> </td"
	.ascii	"><td style=\"color:rgb(51, 51, 255); font-family:Ve"
	.ascii	"rdana; font-weight:bold; font-style:italic; font-si"
	.ascii	"ze: 30px\">\000"
	.section	.rodata.HEADER3,"a"
	.align	2
	.set	.LANCHOR6,. + 0
	.type	HEADER3, %object
	.size	HEADER3, 216
HEADER3:
	.ascii	"</td></tr></table><hr style=\"width:100%;height:2px"
	.ascii	"\"><span style=\"font-weight:bold\"></span><span st"
	.ascii	"yle=\"font-weight:bold\"><table style=\"width:961px"
	.ascii	";height:30px\" border=\"0\" cellpadding=\"2\" cells"
	.ascii	"pacing=\"10\"><tbody><tr>\000"
	.section	.rodata.HIT_COUNTER,"a"
	.align	2
	.set	.LANCHOR10,. + 0
	.type	HIT_COUNTER, %object
	.size	HIT_COUNTER, 27
HIT_COUNTER:
	.ascii	"Number of page hits:&nbsp;\000"
	.section	.rodata.PageErr.13732,"a"
	.align	2
	.set	.LANCHOR12,. + 0
	.type	PageErr.13732, %object
	.size	PageErr.13732, 46
PageErr.13732:
	.ascii	"<br><br> No PageType implementation found<br>\000"
	.section	.rodata.REFRESH_d,"a"
	.align	2
	.set	.LANCHOR4,. + 0
	.type	REFRESH_d, %object
	.size	REFRESH_d, 41
REFRESH_d:
	.ascii	"<meta http-equiv=\"refresh\" content=\"%d\">\000"
	.section	.rodata.TBL_POSTFIX,"a"
	.align	2
	.set	.LANCHOR9,. + 0
	.type	TBL_POSTFIX, %object
	.size	TBL_POSTFIX, 112
TBL_POSTFIX:
	.ascii	"</tr></tbody></table><br></span><span style=\"font-"
	.ascii	"weight:bold\"></span><small><span style=\"font-fami"
	.ascii	"ly:Verdana\">\000"
	.section	.rodata.TITLE_s,"a"
	.align	2
	.set	.LANCHOR3,. + 0
	.type	TITLE_s, %object
	.size	TITLE_s, 30
TITLE_s:
	.ascii	"<html><head><title>%s</title>\000"
	.section	.rodata.WebPages,"a"
	.align	2
	.set	.LANCHOR2,. + 0
	.type	WebPages, %object
	.size	WebPages, 160
WebPages:
	.word	.LC50
	.word	.LC51
	.word	.LC52
	.word	0
	.word	0
	.byte	0
	.space	3
	.word	.LC53
	.space	4
	.word	.LC54
	.word	.LC55
	.word	.LC56
	.word	5
	.word	TaskListPageHits
	.byte	1
	.space	3
	.word	HtmlTaskList
	.word	0
	.word	.LC57
	.word	.LC57
	.word	.LC58
	.word	0
	.word	0
	.byte	1
	.space	3
	.word	HtmlSettingsCM7
	.word	HtmlSetCM7
	.word	.LC59
	.word	.LC59
	.word	.LC60
	.word	0
	.word	0
	.byte	1
	.space	3
	.word	HtmlSettingsCM4
	.word	HtmlSetCM4
	.word	.LC61
	.word	.LC62
	.word	.LC63
	.word	0
	.word	0
	.byte	1
	.space	3
	.word	HtmlShowEthIf
	.word	0
	.section	.rodata.cm4_header,"a"
	.align	2
	.set	.LANCHOR24,. + 0
	.type	cm4_header, %object
	.size	cm4_header, 32
cm4_header:
	.ascii	"<p><b>Core CM4 Settings</b></p>\000"
	.section	.rodata.cm7_header,"a"
	.align	2
	.set	.LANCHOR21,. + 0
	.type	cm7_header, %object
	.size	cm7_header, 32
cm7_header:
	.ascii	"<p><b>Core CM7 Settings</b></p>\000"
	.section	.rodata.ethif_header,"a"
	.align	2
	.set	.LANCHOR20,. + 0
	.type	ethif_header, %object
	.size	ethif_header, 38
ethif_header:
	.ascii	"<p><b>Eth interface statistic</b></p>\000"
	.section	.rodata.form_postfix,"a"
	.align	2
	.set	.LANCHOR23,. + 0
	.type	form_postfix, %object
	.size	form_postfix, 57
form_postfix:
	.ascii	"</table>\012<br><input type=\"submit\" value=\"Subm"
	.ascii	"it\"></form>\000"
	.section	.rodata.form_prefix,"a"
	.align	2
	.set	.LANCHOR22,. + 0
	.type	form_prefix, %object
	.size	form_prefix, 42
form_prefix:
	.ascii	"<form onSubmit=\"before_submit()\"><table>\012\000"
	.ident	"GCC: (GNU) 9.2.1 20191025 (release) [ARM/arm-9-branch revision 277599]"
