# Intelligent-Cane-for-the-Visually-Impaired
This is an NTU Undergraduate Research on Campus (URECA) Project. I started this research as an initial exploration of technologies that may be used to improve the lives of the visually impaired. It was awarded an Excellent Award in the Undergraduate Research Academy 2019(UGRA ASPIRE Conference) hosted by KAIST, and attended by KAIST, HKUST and NTU.

This project aims the develop an Intelligent White Cane for the visually impaired. It has 3 main features 
1) Ultrasonic sensors the detect obstacles and give vibrational alerts via vibrational motors
2) GPS system that periodically updates user's  location unto a Thingspeak server in the case of the user getting lost
3) Object recognition that can identify objects in front of the user upon the push of a button

Feature (1) and (2) are handled by the Particle Photon microcontroller. While the oject recognition model has been trained to recognised 20 objects, it has not been implemented unto a Raspberry Pi as originally intended at the start of the project.

The Particle Photon codes are uploaded. It uses open source libraries for the various sensors. The libraries do not belong to me and credit are due to their original authors.
