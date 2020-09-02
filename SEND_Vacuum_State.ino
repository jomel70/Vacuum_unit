void SEND_Vacuum_State()
{
if (pump_on==1)

{

{
    if (pressure_kPa<vacuum_limit)
      {
          CAN.sendMsgBuf(0x07, 0, 1, pressure_ok);
      }
          else
       {   CAN.sendMsgBuf(0x07, 0, 1, presure_high);
          
       }
}     

}
}
