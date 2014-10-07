.PHONY: clean

CHAPTERS=$(ls text/*.md | sort)

all: ba.pdf

ba.pdf: text/*.md
	pdflatex ba.tex
	bibtex ba || true
	pdflatex ba.tex

ba.tex: ba.tex.in ba.md
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


