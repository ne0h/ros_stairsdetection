all:
	pdflatex main.tex
	pdflatex main.tex

clean:
	rm -rf *.log *.aux *.toc *.loc *.out *.nav *.snm *.run.xml *.bcf

diagram:
	plantuml activitydiagram.uml -tsvg
	rsvg-convert -f pdf -o activitydiagram.pdf activitydiagram.svg