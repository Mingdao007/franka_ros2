TEX=FR3_GRU_ForceControl_Plan

all: verify pdf

verify:
	python3 verify_references.py

pdf:
	latexmk -pdf -xelatex $(TEX).tex

clean:
	latexmk -C
