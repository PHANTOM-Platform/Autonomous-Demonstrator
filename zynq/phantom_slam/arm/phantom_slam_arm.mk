##
## Auto Generated makefile by CodeLite IDE
## any manual changes will be erased      
##
## Debug
ProjectName            :=phantom_slam_arm
ConfigurationName      :=Debug
WorkspacePath          :=/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/demo_workspace
ProjectPath            :=/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/arm
IntermediateDirectory  :=./Debug
OutDir                 := $(IntermediateDirectory)
CurrentFileName        :=
CurrentFilePath        :=
CurrentFileFullPath    :=
User                   :=Anthony Moulds
Date                   :=28/03/19
CodeLitePath           :=/home/anthony/.codelite
LinkerName             :=/usr/bin/arm-linux-gnueabihf-g++
SharedObjectLinkerName :=/usr/bin/arm-linux-gnueabihf-g++ -shared -fPIC
ObjectSuffix           :=.o
DependSuffix           :=.o.d
PreprocessSuffix       :=.i
DebugSwitch            :=-g 
IncludeSwitch          :=-I
LibrarySwitch          :=-l
OutputSwitch           :=-o 
LibraryPathSwitch      :=-L
PreprocessorSwitch     :=-D
SourceSwitch           :=-c 
OutputFile             :=$(IntermediateDirectory)/$(ProjectName)
Preprocessors          :=$(PreprocessorSwitch)MAX_EXT_API_CONNECTIONS=255 $(PreprocessorSwitch)NON_MATLAB_PARSING 
ObjectSwitch           :=-o 
ArchiveOutputSwitch    := 
PreprocessOnlySwitch   :=-E
ObjectsFileList        :="phantom_slam_arm.txt"
PCHCompileFlags        :=
MakeDirCommand         :=mkdir -p
LinkOptions            :=  
IncludePath            :=  $(IncludeSwitch). $(IncludeSwitch). $(IncludeSwitch)/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/core $(IncludeSwitch)/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/vrep $(IncludeSwitch)/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src 
IncludePCH             := 
RcIncludePath          := 
Libs                   := $(LibrarySwitch)pthread $(LibrarySwitch)rt 
ArLibs                 :=  "pthread" "rt" 
LibPath                := $(LibraryPathSwitch). 

##
## Common variables
## AR, CXX, CC, AS, CXXFLAGS and CFLAGS can be overriden using an environment variables
##
AR       := /usr/bin/arm-linux-gnueabihf-ar rcu
CXX      := /usr/bin/arm-linux-gnueabihf-g++
CC       := /usr/bin/arm-linux-gnueabihf-gcc
CXXFLAGS :=  -g -O0 -Wall $(Preprocessors)
CFLAGS   :=  -g -O0 -Wall -static $(Preprocessors)
ASFLAGS  := 
AS       := /usr/bin/arm-linux-gnueabihf-as


##
## User defined environment variables
##
CodeLiteDir:=/usr/share/codelite
Objects0=$(IntermediateDirectory)/up_src_core_CoreSLAM_ext.c$(ObjectSuffix) $(IntermediateDirectory)/up_src_vrep_extApiPlatform.c$(ObjectSuffix) $(IntermediateDirectory)/up_src_vrep_shared_memory.c$(ObjectSuffix) $(IntermediateDirectory)/up_src_core_CoreSLAM_state.c$(ObjectSuffix) $(IntermediateDirectory)/up_src_core_CoreSLAM_loop_closing.c$(ObjectSuffix) $(IntermediateDirectory)/up_src_phantom_slam_host.c$(ObjectSuffix) $(IntermediateDirectory)/up_src_core_CoreSLAM_random.c$(ObjectSuffix) $(IntermediateDirectory)/up_src_vrep_extApi.c$(ObjectSuffix) $(IntermediateDirectory)/up_src_core_CoreSLAM.c$(ObjectSuffix) 



Objects=$(Objects0) 

##
## Main Build Targets 
##
.PHONY: all clean PreBuild PrePreBuild PostBuild MakeIntermediateDirs
all: $(OutputFile)

$(OutputFile): $(IntermediateDirectory)/.d $(Objects) 
	@$(MakeDirCommand) $(@D)
	@echo "" > $(IntermediateDirectory)/.d
	@echo $(Objects0)  > $(ObjectsFileList)
	$(LinkerName) $(OutputSwitch)$(OutputFile) @$(ObjectsFileList) $(LibPath) $(Libs) $(LinkOptions)

MakeIntermediateDirs:
	@test -d ./Debug || $(MakeDirCommand) ./Debug


$(IntermediateDirectory)/.d:
	@test -d ./Debug || $(MakeDirCommand) ./Debug

PreBuild:


##
## Objects
##
$(IntermediateDirectory)/up_src_core_CoreSLAM_ext.c$(ObjectSuffix): ../src/core/CoreSLAM_ext.c $(IntermediateDirectory)/up_src_core_CoreSLAM_ext.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/core/CoreSLAM_ext.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/up_src_core_CoreSLAM_ext.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/up_src_core_CoreSLAM_ext.c$(DependSuffix): ../src/core/CoreSLAM_ext.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/up_src_core_CoreSLAM_ext.c$(ObjectSuffix) -MF$(IntermediateDirectory)/up_src_core_CoreSLAM_ext.c$(DependSuffix) -MM ../src/core/CoreSLAM_ext.c

$(IntermediateDirectory)/up_src_core_CoreSLAM_ext.c$(PreprocessSuffix): ../src/core/CoreSLAM_ext.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/up_src_core_CoreSLAM_ext.c$(PreprocessSuffix) ../src/core/CoreSLAM_ext.c

$(IntermediateDirectory)/up_src_vrep_extApiPlatform.c$(ObjectSuffix): ../src/vrep/extApiPlatform.c $(IntermediateDirectory)/up_src_vrep_extApiPlatform.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/vrep/extApiPlatform.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/up_src_vrep_extApiPlatform.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/up_src_vrep_extApiPlatform.c$(DependSuffix): ../src/vrep/extApiPlatform.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/up_src_vrep_extApiPlatform.c$(ObjectSuffix) -MF$(IntermediateDirectory)/up_src_vrep_extApiPlatform.c$(DependSuffix) -MM ../src/vrep/extApiPlatform.c

$(IntermediateDirectory)/up_src_vrep_extApiPlatform.c$(PreprocessSuffix): ../src/vrep/extApiPlatform.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/up_src_vrep_extApiPlatform.c$(PreprocessSuffix) ../src/vrep/extApiPlatform.c

$(IntermediateDirectory)/up_src_vrep_shared_memory.c$(ObjectSuffix): ../src/vrep/shared_memory.c $(IntermediateDirectory)/up_src_vrep_shared_memory.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/vrep/shared_memory.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/up_src_vrep_shared_memory.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/up_src_vrep_shared_memory.c$(DependSuffix): ../src/vrep/shared_memory.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/up_src_vrep_shared_memory.c$(ObjectSuffix) -MF$(IntermediateDirectory)/up_src_vrep_shared_memory.c$(DependSuffix) -MM ../src/vrep/shared_memory.c

$(IntermediateDirectory)/up_src_vrep_shared_memory.c$(PreprocessSuffix): ../src/vrep/shared_memory.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/up_src_vrep_shared_memory.c$(PreprocessSuffix) ../src/vrep/shared_memory.c

$(IntermediateDirectory)/up_src_core_CoreSLAM_state.c$(ObjectSuffix): ../src/core/CoreSLAM_state.c $(IntermediateDirectory)/up_src_core_CoreSLAM_state.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/core/CoreSLAM_state.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/up_src_core_CoreSLAM_state.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/up_src_core_CoreSLAM_state.c$(DependSuffix): ../src/core/CoreSLAM_state.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/up_src_core_CoreSLAM_state.c$(ObjectSuffix) -MF$(IntermediateDirectory)/up_src_core_CoreSLAM_state.c$(DependSuffix) -MM ../src/core/CoreSLAM_state.c

$(IntermediateDirectory)/up_src_core_CoreSLAM_state.c$(PreprocessSuffix): ../src/core/CoreSLAM_state.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/up_src_core_CoreSLAM_state.c$(PreprocessSuffix) ../src/core/CoreSLAM_state.c

$(IntermediateDirectory)/up_src_core_CoreSLAM_loop_closing.c$(ObjectSuffix): ../src/core/CoreSLAM_loop_closing.c $(IntermediateDirectory)/up_src_core_CoreSLAM_loop_closing.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/core/CoreSLAM_loop_closing.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/up_src_core_CoreSLAM_loop_closing.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/up_src_core_CoreSLAM_loop_closing.c$(DependSuffix): ../src/core/CoreSLAM_loop_closing.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/up_src_core_CoreSLAM_loop_closing.c$(ObjectSuffix) -MF$(IntermediateDirectory)/up_src_core_CoreSLAM_loop_closing.c$(DependSuffix) -MM ../src/core/CoreSLAM_loop_closing.c

$(IntermediateDirectory)/up_src_core_CoreSLAM_loop_closing.c$(PreprocessSuffix): ../src/core/CoreSLAM_loop_closing.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/up_src_core_CoreSLAM_loop_closing.c$(PreprocessSuffix) ../src/core/CoreSLAM_loop_closing.c

$(IntermediateDirectory)/up_src_phantom_slam_host.c$(ObjectSuffix): ../src/phantom_slam_host.c $(IntermediateDirectory)/up_src_phantom_slam_host.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/phantom_slam_host.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/up_src_phantom_slam_host.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/up_src_phantom_slam_host.c$(DependSuffix): ../src/phantom_slam_host.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/up_src_phantom_slam_host.c$(ObjectSuffix) -MF$(IntermediateDirectory)/up_src_phantom_slam_host.c$(DependSuffix) -MM ../src/phantom_slam_host.c

$(IntermediateDirectory)/up_src_phantom_slam_host.c$(PreprocessSuffix): ../src/phantom_slam_host.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/up_src_phantom_slam_host.c$(PreprocessSuffix) ../src/phantom_slam_host.c

$(IntermediateDirectory)/up_src_core_CoreSLAM_random.c$(ObjectSuffix): ../src/core/CoreSLAM_random.c $(IntermediateDirectory)/up_src_core_CoreSLAM_random.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/core/CoreSLAM_random.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/up_src_core_CoreSLAM_random.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/up_src_core_CoreSLAM_random.c$(DependSuffix): ../src/core/CoreSLAM_random.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/up_src_core_CoreSLAM_random.c$(ObjectSuffix) -MF$(IntermediateDirectory)/up_src_core_CoreSLAM_random.c$(DependSuffix) -MM ../src/core/CoreSLAM_random.c

$(IntermediateDirectory)/up_src_core_CoreSLAM_random.c$(PreprocessSuffix): ../src/core/CoreSLAM_random.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/up_src_core_CoreSLAM_random.c$(PreprocessSuffix) ../src/core/CoreSLAM_random.c

$(IntermediateDirectory)/up_src_vrep_extApi.c$(ObjectSuffix): ../src/vrep/extApi.c $(IntermediateDirectory)/up_src_vrep_extApi.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/vrep/extApi.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/up_src_vrep_extApi.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/up_src_vrep_extApi.c$(DependSuffix): ../src/vrep/extApi.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/up_src_vrep_extApi.c$(ObjectSuffix) -MF$(IntermediateDirectory)/up_src_vrep_extApi.c$(DependSuffix) -MM ../src/vrep/extApi.c

$(IntermediateDirectory)/up_src_vrep_extApi.c$(PreprocessSuffix): ../src/vrep/extApi.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/up_src_vrep_extApi.c$(PreprocessSuffix) ../src/vrep/extApi.c

$(IntermediateDirectory)/up_src_core_CoreSLAM.c$(ObjectSuffix): ../src/core/CoreSLAM.c $(IntermediateDirectory)/up_src_core_CoreSLAM.c$(DependSuffix)
	$(CC) $(SourceSwitch) "/home/anthony/PHANTOM/Demo/Robotics/phantom_slam/src/core/CoreSLAM.c" $(CFLAGS) $(ObjectSwitch)$(IntermediateDirectory)/up_src_core_CoreSLAM.c$(ObjectSuffix) $(IncludePath)
$(IntermediateDirectory)/up_src_core_CoreSLAM.c$(DependSuffix): ../src/core/CoreSLAM.c
	@$(CC) $(CFLAGS) $(IncludePath) -MG -MP -MT$(IntermediateDirectory)/up_src_core_CoreSLAM.c$(ObjectSuffix) -MF$(IntermediateDirectory)/up_src_core_CoreSLAM.c$(DependSuffix) -MM ../src/core/CoreSLAM.c

$(IntermediateDirectory)/up_src_core_CoreSLAM.c$(PreprocessSuffix): ../src/core/CoreSLAM.c
	$(CC) $(CFLAGS) $(IncludePath) $(PreprocessOnlySwitch) $(OutputSwitch) $(IntermediateDirectory)/up_src_core_CoreSLAM.c$(PreprocessSuffix) ../src/core/CoreSLAM.c


-include $(IntermediateDirectory)/*$(DependSuffix)
##
## Clean
##
clean:
	$(RM) -r ./Debug/


