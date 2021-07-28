/*
 * z180dbg.h - simple Z180 tracer
 *
 * Copyright (c) Michal Tomek 2018-2019 <mtdev79b@gmail.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 *  02111-1307  USA.
 *
 */

unsigned int g_quit = 0;
unsigned long long instrcnt = 0;
unsigned long long starttrace = -1LL;

void do_timers();

UINT8 debugger_getmem(device_t *device, offs_t addr) {
	UINT8 ROMBR, RAMUBR, RAMLBR, SCR;
	UINT8 *mem = NULL;
	cpu_translate_z180(device,AS_PROGRAM,0,&addr);
	if (((struct z180_device *)device)->m_type == Z180_TYPE_Z182) {
		ROMBR = cpu_get_state_z180(device,Z182_ROMBR);
		RAMUBR = cpu_get_state_z180(device,Z182_RAMUBR);
		RAMLBR = cpu_get_state_z180(device,Z182_RAMLBR);
		SCR = cpu_get_state_z180(device,Z182_SCR);
		if(!(SCR & 8) && (addr >>12) <= ROMBR)
			mem = ROMARRAY;
		else {
			mem = RAMARRAY;	
			if( (addr >>12) < RAMLBR || RAMUBR < (addr >>12) )
				printf("RAM access outside bounds\n");
		}
	} else {
		mem = RAMARRAY;
	}
	return mem[addr];
}

void debugger_instruction_hook(device_t *device, offs_t curpc) {
	//printf(".");
	char ibuf[20];
	offs_t dres,i;
	char fbuf[10];
	UINT8 ROMBR, RAMUBR, RAMLBR, SCR;
	UINT8 *mem = NULL;


	instrcnt++;
	do_timers();

	if(VERBOSE) {
		cpu_string_export_z180(device,STATE_GENFLAGS,fbuf);
		printf("%s AF=%04X BC=%04X DE=%04X HL=%04X IX=%04X IY=%04X SP=%04X\n",fbuf,
		    cpu_get_state_z180(device,Z180_AF),
			cpu_get_state_z180(device,Z180_BC),
			cpu_get_state_z180(device,Z180_DE),
			cpu_get_state_z180(device,Z180_HL),
			cpu_get_state_z180(device,Z180_IX),
			cpu_get_state_z180(device,Z180_IY),
			cpu_get_state_z180(device,Z180_SP));
		cpu_translate_z180(device,AS_PROGRAM,0,&curpc);
		if (((struct z180_device *)device)->m_type == Z180_TYPE_Z182) {
			ROMBR = cpu_get_state_z180(device,Z182_ROMBR);
			RAMUBR = cpu_get_state_z180(device,Z182_RAMUBR);
			RAMLBR = cpu_get_state_z180(device,Z182_RAMLBR);
			SCR = cpu_get_state_z180(device,Z182_SCR);
			if(!(SCR & 8) && (curpc >>12) <= ROMBR)
				mem = ROMARRAY;
			else {
				mem = RAMARRAY;	
				if( (curpc >>12) < RAMLBR || RAMUBR < (curpc >>12) )
					printf("Opcode fetch from RAM outside bounds\n");
			}
		}
		else {
			mem = RAMARRAY;
		}
		dres = cpu_disassemble_z180(device,ibuf,curpc,&mem[curpc],&mem[curpc],0);
		printf("%05x: ",curpc);
		for (i=0;i<(dres &DASMFLAG_LENGTHMASK);i++) printf("%02X",mem[curpc+i]);
		for ( ;i<4;i++) {putchar(' ');putchar(' ');}
		printf(" %s",ibuf);
		if (strstr(ibuf,",(hl)")||strstr(ibuf," (hl)")||strstr(ibuf,"ldi")||strstr(ibuf,"ldd"))
			printf("\tm:%02X",debugger_getmem(device, cpu_get_state_z180(device,Z180_HL)));
		else if (strstr(ibuf,",(de)"))
			printf("\tm:%02X",debugger_getmem(device, cpu_get_state_z180(device,Z180_DE)));
		else if (strstr(ibuf,",(bc)"))
			printf("\tm:%02X",debugger_getmem(device, cpu_get_state_z180(device,Z180_BC)));
		else if (strstr(ibuf,",(ix"))
			printf("\tm:%02X",debugger_getmem(device, cpu_get_state_z180(device,Z180_IX)+(int8_t)mem[curpc+2]));
		else if (strstr(ibuf,",(iy"))
			printf("\tm:%02X",debugger_getmem(device, cpu_get_state_z180(device,Z180_IY)+(int8_t)mem[curpc+2]));
		else if (strstr(ibuf,"(sp),"))	  // ex (sp),...
			printf("\tm:%02X",debugger_getmem(device, cpu_get_state_z180(device,Z180_SP)));
		putchar('\n');
		fflush(stdout);
	}
}
