\def\versionno{1.0}
\def\versiondate{2025/06/06}

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{muthesis}[\versiondate \versionno Maynooth University Thesis Class]

% -----------------------------------------------------
% Base document class
% -----------------------------------------------------
\LoadClass[12pt]{report}

% -----------------------------------------------------
% Essential packages — core functionality and styling
% -----------------------------------------------------

\RequirePackage{etoolbox}        % For command patching (\preto, etc.)
\RequirePackage{fancyhdr}        % Custom headers and footers
\RequirePackage{emptypage}       % Suppress headers/footers on blank pages     

\RequirePackage{graphicx}        % Include images
\RequirePackage{booktabs}        % High-quality table rules

\RequirePackage[english]{babel}  % Language and hyphenation rules
\RequirePackage[utf8]{inputenc}  % UTF-8 support (for pdflatex)
\RequirePackage[T1]{fontenc}     % Better font encoding


\RequirePackage{microtype}       % Better typography (kerning, spacing)
\RequirePackage{csquotes}        % Context-sensitive quotation marks

\usepackage{enumitem}           % enumerate numbering
\usepackage{amssymb}
\usepackage{amsmath}
\usepackage{float}              % Better figure placement control
\usepackage{textcomp, gensymb}
\usepackage{wrapfig}
\usepackage{todonotes}
\usepackage{multirow}

% Caption Style
\RequirePackage[labelfont=bf,justification=justified,font=small]{caption} 
\RequirePackage{subcaption} % supcaption

% -----------------------------------------------------
% Bibliography setup - IEEE style
% -----------------------------------------------------
\RequirePackage[numbers,square]{natbib} % Numbered citations with square brackets
\bibliographystyle{IEEEtranN} % IEEE Transactions bibliography style
\setcitestyle{numbers} % Ensure citations appear as numbers

\RequirePackage[
  a4paper,
  left=3cm,
  right=3cm,
  top=3cm,
  bottom=2.5cm,
  headheight=15pt
]{geometry} % Page layout

\RequirePackage{setspace}        % Line spacing
\onehalfspacing


\RequirePackage{xcolor}          % Color support

\RequirePackage{hyperref}        % Hyperlinks
\hypersetup{
  colorlinks   = true,
  linkcolor    = blue!70!black,
  urlcolor     = blue!60!black,
  citecolor    = blue!90!black,
  filecolor    = blue!40!black
}

% Comment these two lines if you don't use any glossary-related commands %
\RequirePackage[acronym,toc]{glossaries}
\makeglossaries  

% Comment these two lines if you don't use any nomenclature-related commands %
\RequirePackage[intoc]{nomencl} 
\makenomenclature

% -----------------------------------------------------
% Thesis metadata commands
% -----------------------------------------------------
\makeatletter
\let\thesistitle\@empty
\let\thesisauthor\@empty
\let\thesisorcid\@empty
\let\thesisdegree\@empty
\let\thesisdate\@empty
\let\thesisresubmissiondate\@empty
\let\thesisfaculty\@empty
\let\thesisdepartment\@empty
\let\thesisuniversity\@empty
\let\thesisjointinstitution\@empty
\let\departmenthead\@empty
\let\thesissupervisor\@empty
\makeatother

\newcommand{\ThesisTitle}[1]{\renewcommand{\thesistitle}{#1}}
\newcommand{\ThesisAuthor}[1]{\renewcommand{\thesisauthor}{#1}}
\newcommand{\ThesisOrcid}[1]{\renewcommand\thesisorcid{#1}}
\newcommand{\ThesisDegree}[1]{\renewcommand{\thesisdegree}{#1}}
\newcommand{\ThesisDate}[1]{\renewcommand{\thesisdate}{#1}}
\newcommand{\ThesisResubmissionDate}[1]{\renewcommand{\thesisresubmissiondate}{#1}}
\newcommand{\ThesisFaculty}[1]{\renewcommand{\thesisfaculty}{#1}}
\newcommand{\ThesisDepartment}[1]{\renewcommand{\thesisdepartment}{#1}}
\newcommand{\ThesisUniversity}[1]{\renewcommand{\thesisuniversity}{#1}}
\newcommand{\ThesisJointInstitution}[1]{\renewcommand{\thesisjointinstitution}{#1}}
\newcommand{\ThesisSupervisor}[1]{\renewcommand{\thesissupervisor}{#1}}
\newcommand{\DepartmentHead}[1]{\renewcommand{\departmenthead}{#1}}

% -----------------------------------------------------
% Title page layout
% -----------------------------------------------------
\makeatletter
\renewcommand{\maketitle}{%
  \begin{titlepage}
    \centering
    \vspace*{2cm}
    % i. Title
    \ifx\thesistitle\@empty\else
      {\LARGE\bfseries \thesistitle\par}\vspace{1.5cm}
    \fi
    % ii. Author
    \ifx\thesisauthor\@empty\else
      {\Large\bfseries \thesisauthor\par}\vspace{0.5cm}
    \fi
    % iii. ORCID
    \ifx\thesisorcid\@empty\else
      {\small ORCID:\ \thesisorcid\par}\vspace{0.5cm}
    \fi
    % iv. Degree
    \ifx\thesisdegree\@empty\else
      {\large A conversion thesis submitted for the degree of\\ {\bfseries \thesisdegree}\par}\vspace{0.75cm}
    \fi
    % v. Department/Faculty and University
    \ifx\thesisuniversity\@empty\else
      {\large \thesisuniversity\\}
    \fi
    \ifx\thesisfaculty\@empty\else
      {\large \thesisfaculty\\}
    \fi
    \ifx\thesisdepartment\@empty\else
      {\large \thesisdepartment\\}\vspace{0.5cm}
    \fi
    \vfill
    % vi. Jointly awarded institution
    \ifx\thesisjointinstitution\@empty\else
      {\large Jointly awarded with \thesisjointinstitution\par}\vspace{0.5cm}
    \fi
    % vii. Submission date
    \ifx\thesisdate\@empty\else
      {\large \thesisdate\par}\vspace{0.5cm}
    \fi
    % viii. Resubmission date
    \ifx\thesisresubmissiondate\@empty\else
      {\large Resubmitted: \thesisresubmissiondate\par}\vspace{0.5cm}
    \fi
    % ix. Supervisers and department head (optional)
    \ifx\departmenthead\@empty\else
      {\large Head of the Department:\ \departmenthead\par}
    \fi
    \ifx\thesissupervisor\@empty\else
      {\large Supervisor:\ \thesissupervisor\par}
    \fi
    \vfill
  \end{titlepage}%
}
\makeatother

% -----------------------------------------------------
% Frontmatter and mainmatter layout
% -----------------------------------------------------
\makeatletter
\newcommand{\frontmatter}{%
  \cleardoublepage         % blank page if coming off an odd page

  \setlength{\parindent}{0pt} % set paragraph indentation
  \setlength{\parskip}{0.5\baselineskip} % set space between paragraphs
  
  \pagenumbering{roman}    % Roman i, ii, …
  \pagestyle{plain}        % plain = page number only
}

\newcommand{\mainmatter}{%
  \cleardoublepage

    
  % Paragraph formatting
  \setlength{\parindent}{0pt}  % set paragraph indentation
  \setlength{\parskip}{0.5\baselineskip} % set space between paragraphs
  

  \pagenumbering{arabic}
  \pagestyle{fancy}
  \fancyhf{}
  \if@twoside  % For twoside layout
   
    % Odd pages (right pages)
    \fancyhead[RO]{\thepage}
    \fancyhead[LO]{\nouppercase{\leftmark}}
    % Even pages (left pages)
    \fancyhead[LE]{\thepage}
    \fancyhead[RE]{\nouppercase{\leftmark}}
  \else
    % For oneside layout
    \fancyhead[L]{\nouppercase{\leftmark}}
    \fancyhead[R]{\thepage}
  \fi
  \renewcommand{\headrulewidth}{0.4pt}
}
\makeatother

% -----------------------------------------------------
% Ensure lists of figures/tables start on new page
% -----------------------------------------------------
\AtBeginDocument{%
  \preto{\listoffigures}{\cleardoublepage}
  \preto{\listoftables}{\cleardoublepage}
}

% -----------------------------------------------------
% Glossary commands
% -----------------------------------------------------
\newcommand{\PrintAcronym}[1]{%
  \input{#1}\glsaddall
  \printglossary[type=\acronymtype,title=Acronyms,toctitle=Acronyms]
}

\newcommand{\PrintGlossary}[1]{%
  \input{#1}
  \printglossary[title=Glossary,toctitle=Glossary]
}


% -----------------------------------------------------
% Nomenclature commands
% -----------------------------------------------------
\newcommand{\PrintNomenclature}[1]{
  \cleardoublepage            
  \phantomsection   % for correct hyperref target
  \renewcommand{\nomname}{Nomenclature}
  \input{#1}            
  \printnomenclature   
}

\endinput
