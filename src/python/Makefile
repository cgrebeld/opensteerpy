INCLUDEPATH = -I../../include
OBJ_DIR = ../../linux/objs_optimized
OBJS += $(wildcard $(OBJ_DIR)/*.o)

.PHONY: all

all: _opensteer.so

opensteer_wrap.cpp: opensteer.i
	swig -c++ -python $(INCLUDEPATH) -o opensteer_wrap.cpp opensteer.i

opensteer_wrap.o: opensteer_wrap.cpp
	gcc -fPIC $(INCLUDEPATH) -c opensteer_wrap.cpp -o opensteer_wrap.o -I/usr/include/python2.5

_opensteer.so: opensteer_wrap.o
	g++ -shared $(LIBPATH) opensteer_wrap.o -o _opensteer.so $(OBJS)

clean:
	rm -rf *_wrap.h *_wrap.cpp *.so *.o opensteer.py *.pyc *~
