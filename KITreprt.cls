% Vorlage für die Studien-, Diplom-, Bachelor-, Masterarbeiten, am HIS
% (c) Stefan Ulbrich, 2012


\NeedsTeXFormat{LaTeX2e}										% We do need LaTeX2e
\ProvidesClass{KITreprt}
\LoadClass[a4paper,twoside,10pt, DIV=12,BCOR=15mm]{scrreprt} % scrartcl

% not using scrhack causes the following warning:
% % Class scrreprt Warning: \float@addtolists detected!
% % (scrreprt)              You should use the features of package `tocbasic'
% % (scrreprt)              instead of \float@addtolists.
% % (scrreprt)              Support for \float@addtolists may be removed from
% % (scrreprt)              `scrreprt' soon .
\usepackage{scrhack}

% Am Ende die Dokumentklassen: scrartcl: für kleine bis mittlere Dokumente,
% scrreprt: für grosse Dokumente, scrbook: für Bücher

%\usepackage{setspace} % für die Titelseite

\ProcessOptions\relax

\RequirePackage{babel}    %Trennungen, Schriftsatz; Neue deutsche Rechtschreibung
\usepackage[T1]{fontenc}       %Umlaute, Sonderzeichen...

\usepackage[utf8x]{inputenc}
\usepackage{graphicx}   
\usepackage{grffile} %The package extends the file name processing of package graphics to sup- port a larger range of file names.
\usepackage{amsmath}            
\usepackage{amssymb}
\usepackage{amsthm}
\usepackage{color}
\usepackage{subfigure}
%\usepackage{bm}  %bold and cursive letters
%\usepackage{bbm} %hollow letters -- for homogeneous coordinates
\usepackage{tikz}
%\usepackage{tikzpagenodes}
\usetikzlibrary{calc}

\usepackage{eso-pic}
\usepackage{booktabs}

\usepackage[scaled=0.92]{helvet} % Different fonts
\usepackage{courier}
\usepackage{mathptmx}


\usepackage{listings}          % Paket um Quellcode darzustellen
\usepackage{lastpage}          % Spezielles Paket um die Seiten zu zählen

%\usepackage{wallpaper}  % Für den Hintergrund der Titelseite


\usepackage[automark]{scrpage2}          

\usepackage{setspace}
%\onehalfspacing	      %1.5 Zeilenabstand; 1 = \singlespacing; 2 = \doublespacing
%\setlength{\parindent}{1em}     % 1em = Grösse, die ein grosses M der aktuellen Schrift



% hyperref userpackage moved down here because hyperref should be loaded as the last package: http://tex.stackexchange.com/questions/16268/warning-with-footnotes-namehfootnote-xx-has-been-referenced-but-does-not-exi
\usepackage[raiselinks=true,
bookmarks=true,
bookmarksopenlevel=1,
bookmarksopen=true,
bookmarksnumbered=true,
hyperindex=true,
plainpages=false,
pdfpagelabels=true,
pdfborder={0 0 0.5},
colorlinks=false,
pagebackref=true,
linkbordercolor={0 0.61 0.50},
urlbordercolor={0 0.61 0.50},		% for same urlcolor
citebordercolor={0 0.61 0.50},]{hyperref}  %{0.57 0.74 0.57}


\definecolor{gray}{rgb}{0.5,0.5,0.5}





\lstloadlanguages{C++, bash,TeX,Python}
\lstset{
language={[LaTeX]TeX},
morekeywords={{\gqq},{includegraphics}}
}

\lstset{%
language=C++,
morekeywords={includegraphics,citet,citep},
showstringspaces=false,
basicstyle=\ttfamily\small,
keywordstyle=\color{blue},
commentstyle=\color{gray},
identifierstyle=\color{black},
stringstyle=\color{red},
captionpos=b,
frame=single,
frameround=tftf,
framesep=0.5cm,
xleftmargin=0.5cm,
xrightmargin=0.5cm,
rulecolor=\color{gray},float=htb,
breaklines=true,
%escapechar=\% %Enter latex commands in listing
}%


\newcommand{\termpaper}{\iflanguage{english}{Term Paper}{Seminararbeit}}
\newcommand{\mastersthesis}{\iflanguage{english}{Master's Thesis}{Masterarbeit}}
\newcommand{\bachelorsthesis}{\iflanguage{english}{Bachelors's Thesis}{Bachelorarbeit}}
\newcommand{\protocol}{\iflanguage{english}{Protocol}{Protokoll}}
\newcommand{\studienarbeit}{\iflanguage{english}{Student Research Project}{Studienarbeit}}
\newcommand{\diplomarbeit}{\iflanguage{english}{Diploma Thesis}{Diplomarbeit}}


\newcommand{\mytitle} 
{\textcolor{red} {Please redefine the \texttt{mytitle} macro \\ \texttt{\Large\textbackslash renewcommand\{\textbackslash mytitle\}\{\dots\}}}}
\newcommand{\mythesis}
{\textcolor{red} {\small Please redefine the \texttt{mythesis} macro, \\ e.g., \texttt{\textbackslash renewcommand\{\textbackslash mythesis\}\{\textbackslash termpaper\}},\\ options: \texttt{\textbackslash mastersthesis, \textbackslash bachelorsthesis, \textbackslash studienarbeit, \textbackslash diplomarbeit}} }


%{\textcolor{red} {\large Please define the \texttt{mythesis} macro:  \texttt{ \textbackslash renewcommand\{\textbackslash mythesis\}\{\dots\}}
%\\ predefined macros are: \textbackslash termpaper, \textbackslash mastersthesis, \textbackslash bachelorsthesis, \textbackslash studienarbeit,
%\textbackslash dioplomarbeit 
%}}
\newcommand{\timestart}
{\textcolor{red} {\small Please redefine the \texttt{timestart} macro: \texttt{ \textbackslash renewcommand\{\textbackslash timestart\}\{\dots\}}}}
\newcommand{\timeend}
{\textcolor{red} {\small Please redefine the \texttt{timeend} macro:  \texttt{ \textbackslash renewcommand\{\textbackslash timeend\}\{\dots\}}}}
\newcommand{\myname}
 {\textcolor{red} {\large Please redefine the \texttt{myname} macro:  \texttt{ \textbackslash newcommand\{\textbackslash myname\}\{\dots\}}}}
\newcommand{\myshorttitle}
{\textcolor{red}{\small Please redefine the \texttt{myshorttitle} macro (may be empty):  \texttt{ \textbackslash newcommand\{\textbackslash myshorttitle\}\{\dots\}}}}
\newcommand{\noadvisors}
{\textcolor{red}{\small You can specify the advisors and/or referees with \\ 
\textbackslash advisor, \textbackslash advisortwo, \textbackslash referee, \textbackslash refereetwo. \\
If none needed:  \texttt{ \textbackslash newcommand\{\textbackslash noadvisors\}\{\}}}}



%Minted ist nett nur es kann kein line break und keine abgrundeten ecken.
%\newminted[KITcpp]{cpp}{fontfamily=helvetica, frame=single, linenos,mathescape,numberblanklines=false q}

\newcommand{\gqq}[1]{\glqq{}#1\grqq{}}
\newcommand{\gq}[1]{\glq{}#1\grq{}}

\newcommand{\myinstitute}{\iflanguage{english}{Institute for Anthropomatics and Robotics}{Institut f\"ur Anthropomatik und Robotik} (IAR)\\ High Perfomance Humanoid Technologies Lab (H\textsuperscript{2}T)}


\ihead{}%[\mytitle]{\myname}
\rehead{ \leftmark}%\chaptername~\thechapter}
\lohead{ \rightmark} 
\ohead{\sf \iflanguage{english}{Page}{Seite} \pagemark}
\ifoot{}%irgendwas}
\cfoot{\mytitle~---~\myshorttitle}%Letzte \"Anderung: \today}
\ofoot{}%Seitenanzahl: \pageref{LastPage}}
\setheadsepline{0.5pt} %Dicke der Trennlinie Kopfzeile - Text
\setheadtopline{1pt}
\setfootsepline{0.5pt} %Dicke der Trennlinie Fusszeile - Text

% Left/Right mark cannot be influenced directly:
\newcommand{\sectionname} {\iflanguage{english}{Section}{Abschnitt}}
\renewcommand{\chaptermark}[1]{\markboth{\textsf{\chaptername\ \thechapter.\ #1}}{}}
\renewcommand{\sectionmark}[1]{\markright{\textsf{\sectionname{} \thesection:~#1}}{}}



\pagestyle{scrheadings} %gemachte Einstellungen anwenden
\renewcommand{\chapterpagestyle}{scrheadings}

\usepackage[round]{natbib}


%\bibliographystyle{dinat}

%\bibliographystyle{abbrvdin}%dinat



\renewcommand{\maketitle}{
\begin{titlepage}
\thispagestyle{empty}
\KOMAoptions{twoside=off}
\definecolor{cffffff}{RGB}{255,255,255}
\definecolor{c231f20}{RGB}{35,31,32}
\definecolor{c00a990}{RGB}{0,169,144}


\begin{tikzpicture}[remember picture,yscale=-1,x=1cm,y=1cm,overlay,shift=(current page.north west)]
\coordinate (BCOR) at (1.5cm,0.cm);
\coordinate (upperleft) at ($(BCOR)+(1.5cm,1.5cm)$);
\coordinate (upperright) at ($(\paperwidth-1.5cm,1.5cm)$);
  \coordinate (lowerleft) at ($(BCOR)+(1.5cm,\paperheight-2.0cm)$);
  \coordinate (lowerright) at ($(\paperwidth-1.5cm,\paperheight-2.0cm)$);
  \coordinate (logo) at ($(upperleft) + (0.75cm,0.75cm)$);
  \coordinate (logotext) at ($(logo) + (0,1.8cm)$);
  \coordinate (footnote) at ($(\paperwidth/2.0,\paperheight-1.5cm) + 0.5*(BCOR)$); % BCOR/2 is included!
  
  \node [outer sep=0pt] at (footnote) {
  \textbf{\textsf{
  \iflanguage{english}{\scriptsize KIT --- University of the State of Baden-Wuerttemberg and National Research Center of the Helmholtz Association}
  {  \scriptsize KIT --- Universit\"at des Landes Baden-Württemberg und nationales Forschungszentrum in
    der Helmholtz-Gesellschaft          }  \hspace{1cm} \large{\textbf{www.kit.edu}}
  }}
  };
  \node[anchor=base west,xshift=-4pt ] at (logotext) {\textsf{\textbf{\footnotesize\iflanguage{english}{Karlsruhe Institute of Technology}{Karlsruher Institut f\"ur Technologie}  }}};
  \draw[thick,black,rounded corners=0.4cm]
  (upperleft) -- (upperright) -- (lowerright); 
    \draw[thick,black,rounded corners=0.4cm]
  (upperleft) -- (lowerleft) -- (lowerright); 
    \node [text width = \paperwidth-5cm] at ($(current page.center)+0.5*(BCOR)$) {
%
%
%
\sffamily
\bfseries
%\ThisCenterWallPaper{1.0}{images/titlepage}
~\\[3cm]
\begin{center}
\Huge
\begin{spacing}{1.3}
\mytitle \\
\Large
\myshorttitle
\end{spacing}
~\\[0cm]
\Large 
\mythesis
\\
\iflanguage{english}{of}{von} \\[0.5cm]
\Huge 
 \myname
 \\[1cm]
\large
\iflanguage{english}{At the Department of Informatics}{An der Fakult\"at f\"ur Informatik} \\
\myinstitute \\[4cm]
 \ifundefinedorrelax{referee}{
  \ifundefinedorrelax{refereetwo}{
  \ifundefinedorrelax{advisor}{
  \ifundefinedorrelax{advisortwo}{ \noadvisors } {}
   } {}
   } {}
   } {}
  
%\begin{tabular}[ht]{p{0.4\textwidth}p{0.5\textwidth}}
\begin{tabular}[ht]{rl}
   \ifundefinedorrelax{referee}{} 
  { \iflanguage{english}{Primary referee}{Erstgutachter}:  & \referee\\}
  \ifundefinedorrelax{refereetwo}{} 
  { \iflanguage{english}{Secondary referee}{Zweitgutachter}:   & \refereetwo\\}
  \ifundefinedorrelax{advisor}{}
  {\iflanguage{english}{Advisor}{Betreuender Mitarbeiter}: &  \advisor\\}
  \ifundefinedorrelax{advisortwo}{}
  {\iflanguage{english}{Second advisor}{Zweiter betreuender Mitarbeiter}:   & \advisortwo\\}
\end{tabular}\\[2cm]
\large 
\vfill
\iflanguage{english}{Duration}{Bearbeitungszeit}:
\timestart \hspace*{0.25cm} -- \hspace*{0.25cm} \timeend
\end{center}
%    
%    
%   
  };
   
   %The logo exported from Inksape    
    \begin{scope}[shift=(logo),scale=0.12]

  \path[fill=c231f20,even odd rule] (22.0818,12.6820) -- (18.2146,12.6820) --
    (13.6183,7.7472) -- (13.6183,4.9217) -- (18.0714,-0.0000) -- (22.0557,-0.0000)
    -- (16.9256,6.3800) -- (22.0818,12.6820) -- cycle(22.6677,0.0129) --
    (26.1442,0.0129) -- (26.1442,12.6820) -- (22.6677,12.6820) -- (22.6677,0.0129)
    -- cycle(35.3499,0.0129) -- (26.9776,0.0129) -- (26.9776,2.8644) --
    (29.4255,2.8644) -- (29.4255,12.6952) -- (32.9020,12.6952) -- (32.9020,2.8644)
    -- (35.3499,2.8644) -- (35.3499,0.0129);
  \path[fill=c231f20,nonzero rule] (12.7850,12.6820) -- (12.7850,12.6820);
  \path[fill=c231f20,nonzero rule] (12.7850,12.6820) -- (12.7850,12.6820);
  \path[fill=c00a990,even odd rule] (2.8763,4.6484) .. controls (2.2122,5.4557)
    and (1.6393,6.3411) .. (1.1966,7.2914) -- (12.7850,12.6820) --
    (2.8763,4.6484);
  \path[fill=c00a990,even odd rule] (12.7850,12.6820) -- (7.3814,1.1847) ..
    controls (6.4179,1.6276) and (5.5325,2.1743) .. (4.7252,2.8255) --
    (12.7850,12.6820);
  \path[fill=c231f20,even odd rule] (12.7850,12.6820) -- (12.7980,0.0129) ..
    controls (12.7590,0.0129) and (12.7329,0.0129) .. (12.6939,0.0129) .. controls
    (11.6652,0.0129) and (10.6626,0.1433) .. (9.6991,0.3775) -- (12.7850,12.6820);
  \path[fill=c231f20,even odd rule] (12.7850,12.6820) -- (12.7850,12.6820);
  \path[fill=c231f20,even odd rule] (12.7850,12.6820) -- (12.7850,12.6820);
  \path[fill=c00a990,even odd rule] (0.3633,9.5833) .. controls (0.1028,10.5728)
    and (-0.0274,11.6143) .. (-0.0404,12.6820) -- (12.7850,12.6820) --
    (0.3633,9.5833);

\end{scope}
    
\end{tikzpicture}


\end{titlepage}
\cleardoublepage
}



