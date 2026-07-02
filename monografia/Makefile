# Compila a monografia localmente.
# Uso: make           → pdflatex + bibtex + 2 passadas finais
#      make clean     → remove arquivos intermediários
#      make distclean → remove tudo (inclusive o PDF final)
#      make view      → compila e abre o PDF

TEX := monografia

.PHONY: pdf clean distclean view

pdf:
	pdflatex -interaction=nonstopmode $(TEX).tex > /dev/null
	bibtex $(TEX) > /dev/null
	pdflatex -interaction=nonstopmode $(TEX).tex > /dev/null
	pdflatex -interaction=nonstopmode $(TEX).tex > /dev/null
	@echo "✔ $(TEX).pdf gerado"

clean:
	rm -f *.aux *.bbl *.blg *.log *.out *.toc *.lof *.lot *.brf

distclean: clean
	rm -f $(TEX).pdf

view: pdf
	xdg-open $(TEX).pdf
