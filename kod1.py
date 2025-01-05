#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Şerit Takip Etme
"""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image # kameradan veri alacağımız için gerekli
from geometry_msgs.msg import Twist # hız yayını yapabilmek için de twisti çekmemiz gerekiyor
from cv_bridge import CvBridge # kameradan gelecek mesajları opencv ile görüntüye çevirmek için

class SeritTakip():# kullanacağımız ana class fonksiyonunu tanımladık
    def _init_(self):
        rospy.init_node("serit_takip") # classı kendi tagı ile initiate ettik
        self.bridge = CvBridge() # publisher'a abone olma işlemi için köprü kurduk
        rospy.Subscriber("camera/rgb/image_raw",Image,self.kameraCallback)
        #image_raw topicine abone olucaz,image tipinde veri kullanıcak ve kameracallback fonksiyonuna gideceğiz
        self.pub = rospy.Publisher("cmd_vel",Twist,queue_size = 10)
        #hız yayınlayacağımız publisher olucak bu. 
        #cmd_vel topicini yayınlıcaz 
        #twist tipinde olucak ve mesajın kuyruk büyüklüğü 10 olucak
        self.hiz_mesaji = Twist()
        #hız mesajımızı yayınlıyoruz
        rospy.spin()
        #döngüyü başlatıyoruz
        
    def kameraCallback(self,mesaj):
        img = self.bridge.imgmsg_to_cv2(mesaj,"bgr8")
        #gelen mesajı cv2 bgr8 formatındaki resmine çevirmek için köprü kuruyoruz.
        hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
        #convertcolor fonksiyonu ile bgr resmini hsv formatına çeviriyoruz şerit takibini yapabilmek için
        alt_sari = np.array([20,100,100])
        #saf sarı rengini elde edebilmek için alt ve üst değerini tanılmıyoruz
        ust_sari = np.array([40,255,255])
        #üst sınır için hue(renk tonu) değerini 10 arttırıp diğer saturation ve value değerlerini tavan değer yapıyoruz
        maske = cv2.inRange(hsv,alt_sari,ust_sari)
        #inrange fonksiyonu ile yukarıda tanımladığımız alt ve üst range içerisinde 
        #hsv renkleri ile görüntüyü maskeliyoruz
        sonuc = cv2.bitwise_and(img,img,mask=maske)
        #bu maskeyi orjinal bgr görüntü üzerine uyguluyoruz
        h,w,d = img.shape
        #gelen görüntünün yükseklik,genişlik,derinlik değerlerini alıyoruz
        cv2.circle(img,(int(w/2),int(h/2)),5,(0,0,255),-1)
        #tam ekranı ortalayacak şekilde 
        #yarıçapı 5,kırmızı renkli,içi dolu bir nokta çizdiriyoruz
        M = cv2.moments(maske)
        #siyah-beyaz görüntü üzerinde momentleri buluyoruz
        
        if M['m00'] > 0:
        #moment değeri sıfırdan büyükse hareketi gerçekleştirsin
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            #noktayı tam ortalamak için gerekli olan bölüm
            cv2.circle(img,(cx,cy),5,(255,0,0),-1)
            #orjinal görüntü üzerinde merkez noktası 
            #cx ve cy olmak üzere 5 yarıçaplı,mavi renkli,içi dolu bir nokta çizdiriyoruz
            sapma = cx - w/2
            print(sapma)
            self.hiz_mesaji.linear.x = 0.2
            #çizgisel hızımızı tanımladık
            self.hiz_mesaji.angular.z = -sapma/100 #buraya - yazma nedenimiz sapma sağdaysa sola soldaysa sağa dönmesi için
            #robotu yanlamasına koyduğumuzda da hareketini düzeltip gerçekleştirmesi için
            #açısal hız değeri de girdik.
            self.pub.publish(self.hiz_mesaji)
            #hız mesajımızı yayınladık
        else:
            #moment değeri sıfır olduktan sonra ynai görüntü bittikten sonra dursun
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.0
            #hızı sıfırlıyoruz bu iki satırda
            self.pub.publish(self.hiz_mesaji)
        #buradaki if-else bloğu robotun şerit bittiğinde hareketi bitirmesi için gerekli 
        cv2.imshow("Orjinal",img)
        cv2.imshow("Maske",maske)
        cv2.imshow("Sonuc",sonuc)
        #gözlemleyebilmek için sekmeler halinde imshowu kullanıyoruz
        cv2.waitKey(1)
        #sürekli gösterilmesi için waitkey kullanıyoruz,herhangi bir tuşa bastığımızda kapanır
        
SeritTakip()