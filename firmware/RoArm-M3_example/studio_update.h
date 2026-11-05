#pragma once
#include <Update.h>
#include <mbedtls/sha256.h>
#include <esp_ota_ops.h>

QueueHandle_t serialMaintenanceQueue=nullptr;
bool studioUpdatePaused=false;
bool studioUpdateRunning=false, studioUpdateReady=false;
uint32_t studioUpdateToken=0, studioUpdateLast=0;
size_t studioUpdateSize=0, studioUpdateOffset=0;
String studioUpdateExpected, studioLastHex;
mbedtls_sha256_context studioUpdateHash;
bool studioHashLive=false;

void studioUpdateReply(int type,bool ok,const char *error="") {
  JsonDocument d;d["T"]=type*10+1;d["ok"]=ok;d["error"]=error;
  d["protocol"]="studio-serial-ota-v1";d["token"]=studioUpdateToken;
  d["offset"]=studioUpdateOffset;d["size"]=studioUpdateSize;
  d["running"]=studioUpdateRunning;d["ready"]=studioUpdateReady;d["paused"]=studioUpdatePaused;
  d["sha256"]=studioUpdateExpected;
  d["partition"]=esp_ota_get_running_partition()->label;
  serializeJson(d,Serial);Serial.println();
}
int studioHex(char c){if(c>='0'&&c<='9')return c-'0';if(c>='a'&&c<='f')return c-'a'+10;if(c>='A'&&c<='F')return c-'A'+10;return -1;}
void studioUpdateAbort(){
  if(studioUpdateRunning)Update.abort();
  studioUpdateRunning=false;
  if(studioHashLive){mbedtls_sha256_free(&studioUpdateHash);studioHashLive=false;}
  // Motion remains inhibited until an explicit T623 then T211 lock.
}
void studioUpdateHandle(JsonDocument &c) {
  int type=c["T"]| -1;
  if(type==625){studioUpdateReply(type,true);return;}
  if(type==620){
    if(studioUpdateRunning||studioUpdateReady){studioUpdateReply(type,false,"session already exists");return;}
    size_t size=c["size"]|0;String hash=c["sha256"]|"";
    const esp_partition_t *part=esp_ota_get_next_update_partition(nullptr);
    bool valid=String(c["confirm"]|"")=="UPDATE" && part && size>=32 && size<=part->size && hash.length()==64;
    for(size_t i=0;i<hash.length();i++)valid=valid&&studioHex(hash[i])>=0;
    if(!valid){studioUpdateReply(type,false,"invalid size/hash/confirmation or no OTA slot");return;}
    if(!studioReadServos()){studioUpdateReply(type,false,"servo read failed");return;}
    for(int i=0;i<7;i++)if(abs(servoFeedback[i].speed)>3 || servoFeedback[i].mode!=0){studioUpdateReply(type,false,"servo moving or not position mode");return;}
    studioStopConstantMotion();studioTorqueBlocked=true;studioUpdatePaused=true;
    char old[CMD_MAX_LEN];while(xQueueReceive(cmdQueue,old,0)==pdPASS){}
    if(!Update.begin(size,U_FLASH)){studioUpdatePaused=false;studioUpdateReply(type,false,"Update.begin failed; motion inhibited");return;}
    studioUpdateSize=size;studioUpdateOffset=0;studioUpdateExpected=hash;studioUpdateExpected.toLowerCase();studioLastHex="";
    studioUpdateToken=esp_random();if(!studioUpdateToken)studioUpdateToken=1;
    mbedtls_sha256_init(&studioUpdateHash);studioHashLive=true;
    mbedtls_sha256_starts(&studioUpdateHash,0);
    studioUpdateRunning=true;studioUpdateLast=millis();studioUpdateReply(type,true);return;
  }
  if(!studioUpdateToken || c["token"].as<uint32_t>()!=studioUpdateToken){studioUpdateReply(type,false,"wrong session token");return;}
  if(type==623){
    if(studioUpdateReady){studioUpdateReply(type,false,"image committed; reboot required");return;}
    studioUpdateAbort();studioUpdatePaused=false;studioUpdateReply(type,true);return;
  }
  if(type==624){
    if(!studioUpdateReady || String(c["confirm"]|"")!="REBOOT"){studioUpdateReply(type,false,"no verified image or missing confirmation");return;}
    studioUpdateReply(type,true);Serial.flush();delay(100);esp_restart();return;
  }
  if(type==622 && studioUpdateReady){studioUpdateReply(type,true);return;}
  if(!studioUpdateRunning){studioUpdateReply(type,false,"no running update");return;}
  studioUpdateLast=millis();
  if(type==621){
    String hex=c["hex"]|"";size_t off=c["offset"]|size_t(-1);size_t n=hex.length()/2;
    if(hex.length()%2 || n==0 || n>64){studioUpdateReply(type,false,"invalid chunk length");return;}
    u8 data[64];for(size_t i=0;i<n;i++){int a=studioHex(hex[i*2]),b=studioHex(hex[i*2+1]);if(a<0||b<0){studioUpdateReply(type,false,"invalid hex");return;}data[i]=(a<<4)|b;}
    // A lost ACK can be retried without writing the previous chunk twice.
    if(off+n==studioUpdateOffset && hex==studioLastHex){studioUpdateReply(type,true);return;}
    if(off!=studioUpdateOffset || n>studioUpdateSize-studioUpdateOffset){studioUpdateReply(type,false,"offset or size mismatch");return;}
    if(Update.write(data,n)!=n){studioUpdateAbort();studioUpdateReply(type,false,"flash write failed");return;}
    mbedtls_sha256_update(&studioUpdateHash,data,n);studioUpdateOffset+=n;studioLastHex=hex;studioUpdateReply(type,true);return;
  }
  if(type==622){
    if(studioUpdateOffset!=studioUpdateSize){studioUpdateReply(type,false,"image incomplete");return;}
    u8 digest[32];mbedtls_sha256_finish(&studioUpdateHash,digest);
    char hash[65];for(int i=0;i<32;i++)sprintf(hash+i*2,"%02x",digest[i]);hash[64]=0;
    if(studioUpdateExpected!=hash){studioUpdateAbort();studioUpdateReply(type,false,"SHA256 mismatch");return;}
    if(!Update.end(false)){studioUpdateAbort();studioUpdateReply(type,false,"image validation or activation failed");return;}
    mbedtls_sha256_free(&studioUpdateHash);studioHashLive=false;
    studioUpdateRunning=false;studioUpdateReady=true;studioUpdateReply(type,true);return;
  }
  studioUpdateReply(type,false,"unknown update command");
}
void studioUpdatePoll(){
  char line[CMD_MAX_LEN];
  if(serialMaintenanceQueue && xQueueReceive(serialMaintenanceQueue,line,0)==pdPASS){JsonDocument c;if(!deserializeJson(c,line))studioUpdateHandle(c);}
  if(studioUpdateRunning && uint32_t(millis()-studioUpdateLast)>15000){studioUpdateAbort();studioUpdateReply(623,false,"update timeout; motion inhibited");}
}
