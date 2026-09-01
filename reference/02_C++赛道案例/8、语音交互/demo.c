#include<stdlib.h>
void cd(char *path){
    chdir(path);
}
int main()
{
  cd ("/home/pi/test/music/RaspberryPiSDK-master/Linux_voice_1.109/bin/wav/");
  system(" arecord -d 5 -r 16000 -c 1 -t wav -f S16_LE -D hw:1,0 iflytek02.wav");//录制问题
  
  cd ("/home/pi/test/music/RaspberryPiSDK-master/Linux_voice_1.109/bin/");
  system("./iat_sample");//问题匹配并播报正确答案
}