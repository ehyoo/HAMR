long start_time = millis();
long next_time = millis();
int sin_table[200] = {500,516,531,547,563,578,594,609,624,639,655,669,684,699,713,727,741,755,768,781,794,806,819,831,842,854,864,875,885,895,905,914,922,930,938,946,952,959,965,970,976,980,984,988,991,994,996,998,999,1000,1000,1000,999,998,996,994,991,988,984,980,976,970,965,959,952,946,938,930,922,914,905,895,885,875,864,854,842,831,819,806,794,781,768,755,741,727,713,699,684,669,655,639,624,609,594,578,563,547,531,516,500,484,469,453,437,422,406,391,376,361,345,331,316,301,287,273,259,245,232,219,206,194,181,169,158,146,136,125,115,105,95,86,78,70,62,54,48,41,35,30,24,20,16,12,9,6,4,2,1,0,0,0,1,2,4,6,9,12,16,20,24,30,35,41,48,54,62,70,78,86,95,105,115,125,136,146,158,169,181,194,206,219,232,245,259,273,287,301,316,331,345,361,376,391,406,422,437,453,469,484};
int i;

int send_data = 0;

void setup() {
  Serial.begin(250000);
  Serial.println("Arduino Ready"); //needs to be sent to detect that arduino has initialized
  i = 0;
}

void loop() {
   if(Serial.available()){
      String str = Serial.readString();
      Serial.print(str);
      
      if(str == "send\n"){
        send_data = 1;
        delay(200);
      } 
      else if (str == "stop\n"){
        send_data = 0;
      }
   }

   if(send_data){
       Serial.println("st");
       Serial.println(millis() - start_time); // total time
       Serial.println(random(1, 10));
       Serial.println(sin_table[i]);
       Serial.println(1);
       i = i == 199 ? 0: i + 1;
   }

   while (millis() < next_time);
   next_time = millis() + 10;
}
