// Only allow inlined build:
#ifdef FW_ANYWHERE_STARTUP_INCLUDED


  // Store r10 passed by bootloader as gu32FirmwareAbsPosition, need to use hoop if Cortex-M0
  mov r7, r10
  ldr r2, =gu32FirmwareAbsPosition
  str r7, [r2]

  // Store r11 passed by bootloader as gu32FirmwareOffset, need to use hoop if Cortex-M0
  mov r7, r11
  ldr r2, =gu32FirmwareOffset
  str r7, [r2]

  // Store r12 passed by bootloader as gu32FirmwareAbsOffsetChecksum, need to use hoop if Cortex-M0
  mov r7, r12
  ldr r2, =gu32FirmwareAbsOffsetChecksum
  str r7, [r2]

#ifdef FW_ANYWHERE_USE_VTOR
  // Store vector table RAM being address dynamically so systemconfig can map it
  ldr r7, =__ram_vector_table_begin
  ldr r2, =gu32RamVectorTableBegin
  str r7, [r2]
#endif /* #ifdef FW_ANYWHERE_USE_VTOR */

  // Firmware may be booting as standalone. In that case inspect the checksum
  // and if it does not match, we are most likely running from standalone.
  // Funny thing, Cortex-M0 reset values seem to be like 0xffffffff? Well,
  // checksum in anycase takes care of that correct values are loaded.
  ldr r2, =gu32FirmwareAbsPosition // Load variable address
  ldr r2, [r2] // Load variable data
  ldr r3, =gu32FirmwareOffset // Load variable address
  ldr r3, [r3] // Load variable data
  ldr r4, =gu32FirmwareAbsOffsetChecksum // Load variable address
  ldr r4, [r4] // Load variable data
  movs r1, r2// Calculating the checksum into r1
  eors r1, r1, r3 // r2/gu32FirmwareAbsPosition already there, need only r3/gu32FirmwareOffset
  cmp r1, r4 // Actual compare
  beq BootloadedBootContinue // If match, just do nothing
  // Did not match, so we need to store correct values of gu32FirmwareAbsPosition and gu32FirmwareOffset
  ldr r1, =__flash_begin; // Load variable address
  ldr r2, =gu32FirmwareAbsPosition // Load variable address
  str r1, [r2] // Finally store the new value to ram
  movs r1, #0 // Put zero offset
  ldr r2, =gu32FirmwareOffset // Load firmware offset variable address
  str r1, [r2] // Store zero offset
  // Leave the checksum in memory as it was, even if it was wrong

BootloadedBootContinue:


  // GOT needs to be in RAM in every case
GlobalOffsetTableCopyPatchInit:
  movs r0, #0 // Loop variable
  movs r1, #0 // Pointer (just introduction)

GlobalOffsetTableCopyPatchLoopCond:
  ldr r2, =__flash_global_offset_table_begin // Need global offset table table beginning for pointer
  ldr r3, =__flash_global_offset_table_end // And need end for checking loop
  ldr r4, =gu32FirmwareOffset // Need also data offset variable address
  ldr r4, [r4] // And the actual offset value
  adds r2, r2, r4 // Patching flash global offset table begin to honour offset
  adds r3, r3, r4 // Patching flash global offset table end to honour offset
  adds r1, r0, r2 // Pointer value is loop variable + offsetted flash global offset table begin
  cmp r1, r3 // Compare pointer against global offset table flash end
  bhs GlobalOffsetTableCopyPatchEnd // If getting past limits, go to end

GlobalOffsetTableCopyPatchLoopBody:
  ldr r2, [r1] // Load the actual data via pointer
  ldr r3, =__flash_begin // Need flash begin boundary for checking
  ldr r4, =__flash_end // Need also flash end boundary for checking
  cmp r2, r3 // Comparing loaded data to flash begin
  blo GlobalOffsetTableStoreData // If less than flash begin, jump to store
  cmp r2, r4 // Comparing loaded data to flash end
  bhs GlobalOffsetTableStoreData // If more than or equal to end, jump to store

GlobalOffsetTablePatchData:
  ldr r3, =gu32FirmwareOffset // Need data offset variable address
  ldr r3, [r3] // And then the actual data
  adds r2, r2, r3 // Patch the data

GlobalOffsetTableStoreData:
  ldr r3, =__ram_global_offset_table_begin // Get global offset table begin in ram for ram data pointer
  adds r3, r3, r0 // Add loop variable
  str r2, [r3] // Store the data

GlobalOffsetTableLoopIncrements:
  adds r0, r0, #4 // Increment loop
  b GlobalOffsetTableCopyPatchLoopCond // Jump to loop condition checking

GlobalOffsetTableCopyPatchEnd:
  ldr r0, =__ram_global_offset_table_begin
  mov r9, r0 // Stupid trick to put global offset table location to r9, for Cortex-M0



  // Need to copy and patch vector table in assembly so nobody comes to mess around
VectorTableCopyPatchInit:
  movs r0, #0 // Loop variable
  movs r1, #0 // Pointer (just introduction)

VectorTableCopyPatchLoopCond:
  ldr r2, =__flash_vector_table_begin // Need vector table beginning for pointer
  ldr r3, =__flash_vector_table_end // And need end for checking loop
  ldr r4, =gu32FirmwareOffset // Need also data offset variable address
  ldr r4, [r4] // And the actual offset value
  adds r2, r2, r4 // Patching flash vector table begin to honour offset
  adds r3, r3, r4 // Patching flash vector table end to honour offset
  adds r1, r0, r2 // Pointer value is loop variable + offsetted flash vector table begin
  cmp r1, r3 // Compare pointer against vector table flash end
  bhs VectorTableCopyPatchEnd // If getting past limits, go to end

VectorTableCopyPatchLoopBody:
  ldr r2, [r1] // Load the actual data via pointer
  ldr r3, =__flash_begin // Need flash begin boundary for checking
  ldr r4, =__flash_end // Need also flash end boundary for checking
  cmp r2, r3 // Comparing loaded data to flash begin
  blo VectorTableStoreData // If less than flash begin, jump to store
  cmp r2, r4 // Comparing loaded data to flash end
  bhs VectorTableStoreData // If more than or equal to end, jump to store

VectorTablePatchData:
  ldr r3, =gu32FirmwareOffset // Need data offset variable address
  ldr r3, [r3] // And then the actual data
  adds r2, r2, r3 // Patch the data

VectorTableStoreData:
  ldr r3, =__ram_vector_table_begin // Get vector table begin in ram for ram data pointer
  adds r3, r3, r0 // Add loop variable
  str r2, [r3] // Store the data

VectorTableLoopIncrements:
  adds r0, r0, #4 // Increment loop
  b VectorTableCopyPatchLoopCond // Jump to loop condition checking

VectorTableCopyPatchEnd:



/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  ldr r7, =gu32FirmwareOffset // Load firmware offset variable address
  ldr r7, [r7] // Load the actual firmware offset variable data
  adds r2, r2, r7 // Patch the sidata location with offset
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  // Here we need to check that we are not zeroing out addresses or needed symbols

  ldr r6, =gu32FirmwareAbsPosition // Load address of absolute firmware position variable
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =gu32FirmwareOffset // Load address of firmware offset variable
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

  ldr r6, =gu32FirmwareAbsOffsetChecksum // Load address of firmware position and offset checksum
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away

#ifdef FW_ANYWHERE_USE_VTOR
  ldr r6, =gu32RamVectorTableBegin // Vector table location in RAM
  cmp r2, r6 // Compare with what we are going to zero
  beq FillZerobssSkip // If we should skip zeroing, jump away
#endif /* #ifdef FW_ANYWHERE_USE_VTOR */



  str  r3, [r2] // If not escaped yet, make the store

FillZerobssSkip:
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss


/* Call the clock system initialization function.*/
    bl  SystemInit



// Make our own __libc_init_array
CallPreinitsInit:
  ldr r7, =gu32FirmwareOffset
  ldr r7, [r7]
  ldr r0, =__preinit_array_start
  adds r0, r7
  ldr r1, =__preinit_array_end
  adds r1, r7

CallPreinitsLoopCond:
  cmp r0, r1
  beq CallPreinitsEnd// If same, it is at end, go away

CallPreinitsLoop:
  ldr r5, =__init_array_start
  ldr r4, =__init_array_end // Yes, order is funny to say the least
  ldr r3, [r0]
  push {r0, r1, r2, r3, r4, r5, r6, r7} // Save context because calling externals
  blx r3
  pop {r0, r1, r2, r3, r4, r5, r6, r7} // Retrieve context
  adds r0, r0, #4
  b CallPreinitsLoopCond

CallPreinitsEnd:
  ldr r3, =_init
  adds r3, r7
  ldr r5, =__init_array_start
  adds r5, r7
  ldr r4, =__init_array_end
  adds r4, r7
  push {r0, r1, r2, r3, r4, r5, r6, r7} // Save context because calling externals
  blx r3
  pop {r0, r1, r2, r3, r4, r5, r6, r7} // Retrieve context

CallInitsInit:
  ldr r7, =gu32FirmwareOffset
  ldr r7, [r7]

CallInitsLoopCond:
  cmp r5, r4
  beq CallInitsEnd

CallInitsLoop:
  ldr r3, [r5]
  add r3, r3, r7
  push {r0, r1, r2, r3, r4, r5, r6, r7} // Save context because calling externals
  blx r3
  pop {r0, r1, r2, r3, r4, r5, r6, r7} // Retrieve context
  adds r5, r5, #4
  b CallInitsLoopCond

CallInitsEnd:


#endif /* #ifdef FW_ANYWHERE_STARTUP_INCLUDED */
