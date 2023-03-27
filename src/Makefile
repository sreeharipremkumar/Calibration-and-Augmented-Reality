CC=g++
CFLAGS += -I.
CXXFLAGS = -I/usr/local/include/opencv4
LIBS = `pkg-config --cflags --libs opencv4`
LDFLAGS += -L/usr/local/lib
LDLIBS += -lopencv_xfeatures2d #-ltiff -lpng -ljpeg -lz -lopencv_core -lopencv_highgui -lopencv_video -lopencv_videoio -lopencv_imgcodecs -lopencv_imgproc -lopencv_objdetect 

CALI = cameraCalibration.cpp 

AR = AugmentedReality.cpp

SURF = surfDetect.cpp

all: calibrate Augment surf

%.o: %.cpp # $(DEPS)
	$(CC) $(CXXFLAGS) -c -o $@ $< $(LIBS)

calibrate: $(CALI)
	$(CC) -o  $@ $^ $(LIBS)

Augment: $(AR)
	$(CC) -o  $@ $^ $(LIBS)

surf: $(SURF)
	$(CC) $(CXXFLAGS) -o  $@ $^ $(LIBS) $(LDFLAGS) $(LDLIBS)

clean:
	rm -f *.o *~
