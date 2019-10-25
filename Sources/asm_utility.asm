	.global __get_PSP
__get_PSP: .asmfunc

	MRS R0, PSP	;read the PSP register (the return value needs to be placed into R0)

	BX LR	;return

	.endasmfunc

	.end
