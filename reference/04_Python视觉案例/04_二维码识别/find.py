import cv2
import pyzbar.pyzbar as pyzbar
 
 
def detect():
    camera = cv2.VideoCapture(0)
    while True:
        ret, frame = camera.read()
 
        barcodes = pyzbar.decode(frame)  # 解析摄像头捕获到的所有二维码
 
        data = ''
        # 遍历所有的二维码
        for barcode in barcodes:
            data = barcode.data.decode('utf-8')  # 对数据进行转码
        if data != '':
            with open('data.txt', 'w') as file:
                file.write(data)
            break
        if cv2.waitKey(1) == ord('q'):
            break
        cv2.imshow('', frame)
    camera.release()
    cv2.destroyAllWindows()
 
 
if __name__ == '__main__':
    detect()
