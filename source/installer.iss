#define MyAppVersion "1.0"
#ifndef SourceRoot
  #define SourceRoot ".."
#endif
#ifndef ReleaseOutput
  #define ReleaseOutput "..\.."
#endif

[Setup]
AppId={{BB1807C1-D5CD-43E1-AB2A-E2563D50D8C9}
AppName=RoarmStudio
AppVersion={#MyAppVersion}
AppPublisher=Azucia
DefaultDirName={localappdata}\Programs\RoarmStudio
DefaultGroupName=RoarmStudio
PrivilegesRequired=lowest
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
MinVersion=10.0.17763
OutputDir={#ReleaseOutput}
OutputBaseFilename=RoarmStudio-{#MyAppVersion}-Setup-x64
Compression=lzma2
SolidCompression=yes
WizardStyle=modern
UninstallDisplayIcon={app}\RoarmStudio.exe
CloseApplications=yes
RestartApplications=no
SetupLogging=yes

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"
Name: "chinesesimplified"; MessagesFile: "compiler:Languages\ChineseSimplified.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; Flags: unchecked

[Files]
Source: "{#SourceRoot}\程序\RoarmStudio\RoarmStudio.exe"; DestDir: "{app}"; Flags: ignoreversion
Source: "{#SourceRoot}\程序\RoarmStudio\_internal\*"; DestDir: "{app}\_internal"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#SourceRoot}\assets\*"; DestDir: "{app}\assets"; Flags: ignoreversion
Source: "{#SourceRoot}\licenses\*"; DestDir: "{app}\licenses"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#SourceRoot}\使用说明.md"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\RoarmStudio"; Filename: "{app}\RoarmStudio.exe"; WorkingDir: "{app}"
Name: "{group}\Uninstall RoarmStudio"; Filename: "{uninstallexe}"
Name: "{autodesktop}\RoarmStudio"; Filename: "{app}\RoarmStudio.exe"; WorkingDir: "{app}"; Tasks: desktopicon

[Run]
Filename: "{app}\RoarmStudio.exe"; Description: "Launch RoarmStudio"; Flags: nowait postinstall skipifsilent
