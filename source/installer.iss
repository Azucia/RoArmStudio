#define MyAppVersion "0.4.0"
#ifndef SourceRoot
  #define SourceRoot ".."
#endif
#ifndef ReleaseOutput
  #define ReleaseOutput "..\.."
#endif

[Setup]
AppId={{BB1807C1-D5CD-43E1-AB2A-E2563D50D8C9}
AppName=RoArm Studio
AppVersion={#MyAppVersion}
AppPublisher=RoArm Studio
DefaultDirName={localappdata}\Programs\RoArmStudio
DefaultGroupName=RoArm Studio
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=10.0.17763
OutputDir={#ReleaseOutput}
OutputBaseFilename=RoArmStudio-{#MyAppVersion}-Setup-x64
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
UninstallDisplayIcon={app}\RoArmStudio.exe
CloseApplications=yes
RestartApplications=no
SetupLogging=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "chinesesimplified"; MessagesFile: "compiler:Languages\ChineseSimplified.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; Flags: unchecked

[Files]
Source: "{#SourceRoot}\程序\RoArmStudio\RoArmStudio.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceRoot}\程序\RoArmStudio\_internal\*"; DestDir: "{app}\_internal"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#SourceRoot}\licenses\*"; DestDir: "{app}\licenses"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#SourceRoot}\使用说明.md"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\RoArm Studio"; Filename: "{app}\RoArmStudio.exe"; WorkingDir: "{app}"
Name: "{group}\Uninstall RoArm Studio"; Filename: "{uninstallexe}"
Name: "{autodesktop}\RoArm Studio"; Filename: "{app}\RoArmStudio.exe"; WorkingDir: "{app}"; Tasks: desktopicon

[Run]
Filename: "{app}\RoArmStudio.exe"; Description: "Launch RoArm Studio"; Flags: nowait postinstall skipifsilent
