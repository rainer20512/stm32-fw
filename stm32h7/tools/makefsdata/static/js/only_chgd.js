<script>
var ch = []
function add(e) {
	var n = e.name;
	if(ch.indexOf(n) == -1)
		ch.push(n);
}

function is_changed(n) {
	for(var k = 0; k < ch.length; k++)
		if(n == ch[k]) return n && true;
	return false;
}

function before_submit() {
	var inp = document.getElementsByTagName("input");
	var sel = document.getElementsByTagName("select");
	for(var k = 0; k < inp.length; k++) {
		var n = inp[k].name;
		if(!is_changed(n)) inp[k].disabled = true;
	}
	for(var k = 0; k < sel.length; k++) {
		var n = sel[k].name;
		if(!is_changed(n)) sel[k].disabled = true;
	}
}
</script>