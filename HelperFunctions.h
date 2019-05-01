void user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
    // ESP.wdtDisable();
     unsigned long start = millis();
     do{
       }
    while((millis() - start) < period);
    //ESP.wdtEnable(1000);
}

int8_t user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
    uint8_t i = 0;
    uint8_t c = 0;
    uint8_t data_reg = 0;
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    
    /*while ( i < len)
    {
      Serial.print((uint32_t)&reg_data[i]);
      Serial.print(",");
    }
    Serial.println();
    i = 0;
    */
    Wire.beginTransmission(dev_id);
    Wire.write(reg_addr);
    Wire.endTransmission();

    //Serial.print("Reading register: ");
    //Serial.print(reg_addr,HEX);
    //Serial.print(" with length of: ");
    //Serial.println(len);

    // request bytes from slave device
    //Serial.print("Reciving data: ");
    Wire.requestFrom(dev_id, len);
    if (Wire.available()){
    while ( (Wire.available()) && (i < len))  // slave may send less than requested
    {
      c = Wire.read(); // receive a byte as character
      *reg_data = c;
      *reg_data++;
      i++;
      //Serial.print(c,HEX);
      //Serial.print(",");
    }
    }else{
     // Tdebug += "Failed to reaad the IC2 Bus from"  dev_id;
    }
    //Serial.println("");
    data_reg = *reg_data;
    //Serial.print("Done Reading register: ");
    //Serial.print(reg_addr,HEX);
    
    //Serial.print(" Data is: ");
    /*while ( i < len)
    {
      Serial.print((uint32_t)&reg_data[i]);
      Serial.print(",");
    }*/
    //Serial.println();
    //Serial.println((uint32_t)&reg_data[0],HEX);

    if (reg_addr == 0xF7){
    uint32_t data_msb = (uint32_t)&reg_data[0] << 12;
    uint32_t data_lsb = (uint32_t)&reg_data[1] << 4;
    uint32_t data_xlsb = (uint32_t)&reg_data[2] >> 4;
    uint32_t pressure = data_msb | data_lsb | data_xlsb;

    //Serial.print("Uncomp Pressure is: ");
    //Serial.println(pressure);

    data_msb = (uint32_t)&reg_data[3] << 12;
    data_lsb = (uint32_t)&reg_data[4] << 4;
    data_xlsb = (uint32_t)&reg_data[5] >> 4;
    uint32_t temperature = data_msb | data_lsb | data_xlsb;
    //Serial.print("Uncomp Temerature is: ");
    //Serial.println(temperature);
    }
    return rslt;
}

int8_t user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    //Serial.print("Requested a data write to Address: ");
    //Serial.print(reg_addr,HEX);
    //Serial.print(" with length of: ");
    //Serial.println(len);
    

    Wire.beginTransmission(dev_id);
    Wire.write(reg_addr);
    Wire.write(*reg_data);
    Wire.endTransmission();
    return rslt;
}
