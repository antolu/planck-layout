KEYMAP_DIR=qmk/keyboards/planck/keymaps/antolu

.PHONY: build
build: link
	./qmk.sh compile -j4 -kb planck/rev6 -km antolu

.PHONY: flash
flash: link
# flash command also automatically builds
	./qmk.sh flash -j4 -kb planck/rev6 -km antolu

.PHONY: link
link: unlink
	ln -sf "$(realpath keymap)" "${KEYMAP_DIR}"

.PHONY: unlink
unlink:
	rm -rf  "${KEYMAP_DIR}"

.PHONY: clean
clean: unlink
	./qmk.sh clean
