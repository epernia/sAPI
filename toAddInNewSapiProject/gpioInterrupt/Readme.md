Ver funciones en "pinint_18xx_43xx.h" tiene las funciones de manejo de ISR, ver además funciones STATIC INLINE como decía Martin:


STATIC INLINE void Chip_PININT_SetPinModeEdge(LPC_PIN_INT_T *pPININT, uint32_t pins)
{
    pPININT->ISEL &= ~pins;
}
