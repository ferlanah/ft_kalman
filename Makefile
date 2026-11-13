NAME	= ft_kalman

SRCS	= $(shell find src -name '*.rs') Cargo.toml

# Equivalent of -Wall -Wextra -Werror: every rustc warning is an error.
RUSTFLAGS_ALL = -D warnings
# Static binary (glibc crt-static is only supported on Linux)
ifeq ($(shell uname -s),Linux)
RUSTFLAGS_ALL += -C target-feature=+crt-static
endif

all: $(NAME)

# Depends on the sources only: a second `make` does nothing (no relink).
$(NAME): $(SRCS)
	RUSTFLAGS="$(RUSTFLAGS_ALL)" cargo build --release
	cp target/release/$(NAME) $(NAME)

test:
	RUSTFLAGS="-D warnings" cargo test --release

# 90-minute trajectories against the real stream (about 10 real minutes).
test-long:
	RUSTFLAGS="-D warnings" cargo test --release --test e2e -- --ignored --test-threads=1

# Leak check under valgrind (non-static build).
valgrind:
	./scripts/valgrind.sh

clean:
	cargo clean

fclean: clean
	rm -f $(NAME)

re: fclean all

.PHONY: all test test-long valgrind clean fclean re
