cc        := g++
nvcc      = /usr/local/cuda-10.2/bin/nvcc

cpp_srcs  := $(shell find src -name "*.cpp")
cpp_objs  := $(cpp_srcs:.cpp=.cpp.o)
cpp_objs  := $(cpp_objs:src/%=objs/%)
cpp_mk	  := $(cpp_objs:.cpp.o=.cpp.mk)

cu_srcs	  := $(shell find src -name "*.cu")
cu_objs   := $(cu_srcs:.cu=.cu.o)
cu_objs	  := $(cu_objs:src/%=objs/%)
cu_mk	  := $(cu_objs:.cu.o=.cu.mk)

include_paths := src        \
			/usr/include/opencv4 \
			/usr/include/aarch64-linux-gnu \
			/usr/local/cuda-10.2/include

library_paths := /usr/lib/aarch64-linux-gnu \
			/usr/local/cuda-10.2/lib64

link_librarys := opencv_core opencv_highgui opencv_imgproc opencv_videoio opencv_imgcodecs \
			nvinfer nvinfer_plugin nvonnxparser \
			cuda cublas cudart cudnn \
			stdc++ dl

empty		  :=
export_path   := $(subst $(empty) $(empty),:,$(library_paths))

run_paths     := $(foreach item,$(library_paths),-Wl,-rpath=$(item))
include_paths := $(foreach item,$(include_paths),-I$(item))
library_paths := $(foreach item,$(library_paths),-L$(item))
link_librarys := $(foreach item,$(link_librarys),-l$(item))

cpp_compile_flags := -std=c++11 -fPIC -w -g -pthread -fopenmp -O0
cu_compile_flags  := -std=c++11 -g -w -O0 -Xcompiler "$(cpp_compile_flags)"
link_flags        := -pthread -fopenmp -Wl,-rpath='$$ORIGIN'

cpp_compile_flags += $(include_paths)
cu_compile_flags  += $(include_paths)
link_flags        += $(library_paths) $(link_librarys) $(run_paths)

ifneq ($(MAKECMDGOALS), clean)
-include $(cpp_mk) $(cu_mk)
endif

pro	   := workspace/pro
expath := library_path.txt

library_path.txt : 
	@echo LD_LIBRARY_PATH=$(export_path):"$$"LD_LIBRARY_PATH > $@

workspace/pro : $(cpp_objs) $(cu_objs)
		@echo Link $@
		@mkdir -p $(dir $@)
		@$(cc) $^ -o $@ $(link_flags)

objs/%.cpp.o : src/%.cpp
	@echo Compile CXX $<
	@mkdir -p $(dir $@)
	@$(cc) -c $< -o $@ $(cpp_compile_flags)

objs/%.cu.o : src/%.cu
	@echo Compile CUDA $<
	@mkdir -p $(dir $@)
	@$(nvcc) -c $< -o $@ $(cu_compile_flags)

objs/%.cpp.mk : src/%.cpp
	@echo Compile depends CXX $<
	@mkdir -p $(dir $@)
	@$(cc) -M $< -MF $@ -MT $(@:.cpp.mk=.cpp.o) $(cpp_compile_flags)
	
objs/%.cu.mk : src/%.cu
	@echo Compile depends CUDA $<
	@mkdir -p $(dir $@)
	@$(nvcc) -M $< -MF $@ -MT $(@:.cu.mk=.cu.o) $(cu_compile_flags)

run   : workspace/pro
		  @cd workspace && ./pro

clean :
	@rm -rf objs workspace/pro
	@rm -rf library_path.txt
	@rm -rf workspace/Result.jpg

# 导出符号，使得运行时能够链接上
export LD_LIBRARY_PATH:=$(export_path):$(LD_LIBRARY_PATH)
