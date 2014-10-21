.PHONY: clean

CHAPTERS=./text/*.md

all: ba.pdf

ba.pdf: ba.tex images/*.png ba.bib KITreprt.cls
	pdflatex ba.tex
	bibtex ba || true
	pdflatex ba.tex
	pdflatex ba.tex

ba.tex: ba.tex.in text/*.md
	pandoc --chapters --template ba.tex.in ${CHAPTERS} -o ba.tex

clean:
	rm -f *.aux
	rm -f *.log
	rm -f *.toc
	rm -f *.nav
	rm -f *.out
	rm -f *.snm
	rm -f *.bbl
	rm -f *.blg
	rm -f *.brf
	rm -f *.pdf


