# Microsoft Developer Studio Project File - Name="IKSolver" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=IKSolver - Win32 Debug
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "IKSolver.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "IKSolver.mak" CFG="IKSolver - Win32 Debug"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "IKSolver - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE "IKSolver - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
RSC=rc.exe

!IF  "$(CFG)" == "IKSolver - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "Release"
# PROP BASE Intermediate_Dir "Release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "Release"
# PROP Intermediate_Dir "Release"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /GX /O2 /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /Yu"stdafx.h" /FD /c
# ADD CPP /nologo /W3 /GX /D "WIN32" /D "NDEBUG" /D "_CONSOLE" /D "_MBCS" /Yu"stdafx.h" /FD /c
# SUBTRACT CPP /O<none>
# ADD BASE RSC /l 0x404 /d "NDEBUG"
# ADD RSC /l 0x404 /d "NDEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /machine:I386
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386

!ELSEIF  "$(CFG)" == "IKSolver - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "Debug"
# PROP BASE Intermediate_Dir "Debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "Debug"
# PROP Intermediate_Dir "Debug"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /Yu"stdafx.h" /FD /GZ /c
# ADD CPP /nologo /W3 /Gm /GX /ZI /Od /D "WIN32" /D "_DEBUG" /D "_CONSOLE" /D "_MBCS" /Yu"stdafx.h" /FD /GZ /c
# ADD BASE RSC /l 0x404 /d "_DEBUG"
# ADD RSC /l 0x404 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept
# ADD LINK32 kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib kernel32.lib user32.lib gdi32.lib winspool.lib comdlg32.lib advapi32.lib shell32.lib ole32.lib oleaut32.lib uuid.lib odbc32.lib odbccp32.lib /nologo /subsystem:console /debug /machine:I386 /pdbtype:sept

!ENDIF 

# Begin Target

# Name "IKSolver - Win32 Release"
# Name "IKSolver - Win32 Debug"
# Begin Group "Source Files"

# PROP Default_Filter "cpp;c;cxx;rc;def;r;odl;idl;hpj;bat"
# Begin Source File

SOURCE=.\MotionCurve\ControlPoint.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\ControlPointSet.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\CubicBSplineCurve.cpp
# End Source File
# Begin Source File

SOURCE=.\Cylinder.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\DOFName.cpp
# End Source File
# Begin Source File

SOURCE=.\IKSolver.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Inequality.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\KeyFrame.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\MarkerSeq.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Matrix.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\MotionData.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\MotionMarker.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Path.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\PathPoint.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\PathPointSet.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Point2D.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Point2DSet.cpp
# End Source File
# Begin Source File

SOURCE=.\StdAfx.cpp
# ADD CPP /Yc"stdafx.h"
# End Source File
# Begin Source File

SOURCE=.\Tuple.cpp
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Vector3D.cpp
# End Source File
# Begin Source File

SOURCE=.\Weight.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter "h;hpp;hxx;hm;inl"
# Begin Source File

SOURCE=.\MotionCurve\ControlPoint.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\ControlPointSet.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\CubicBSplineCurve.h
# End Source File
# Begin Source File

SOURCE=.\Cylinder.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\DOFName.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\KeyFrame.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\MarkerSeq.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Matrix.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\MotionData.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\MotionMarker.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Path.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\PathPoint.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\PathPointSet.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Point2D.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Point2DSet.h
# End Source File
# Begin Source File

SOURCE=.\StdAfx.h
# End Source File
# Begin Source File

SOURCE=.\Tuple.h
# End Source File
# Begin Source File

SOURCE=.\MotionCurve\Vector3D.h
# End Source File
# Begin Source File

SOURCE=.\Weight.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter "ico;cur;bmp;dlg;rc2;rct;bin;rgs;gif;jpg;jpeg;jpe"
# End Group
# Begin Source File

SOURCE=.\ReadMe.txt
# End Source File
# End Target
# End Project
