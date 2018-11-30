echo_title() {
    echo
    echo -e '\e[34m' $1 '\e[39m'
    echo
}

run_cmd() {
    cmd=$1.sh
    shift
    . $cmd $@ || die "$cmd $@ died with error code $?"
}

cmd_exists() {
    command -v $1 > /dev/null 2>&1
}

die() {
    echo $1
    exit 1
}
