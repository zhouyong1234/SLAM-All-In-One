/*!
  Highlight.js v11.0.1 (git: 1cf31f015d)
  (c) 2006-2021 Ivan Sagalaev and other contributors
  License: BSD-3-Clause
 */
  var hljs=function(){"use strict";var e={exports:{}};function t(e){
    return e instanceof Map?e.clear=e.delete=e.set=()=>{
    throw Error("map is read-only")}:e instanceof Set&&(e.add=e.clear=e.delete=()=>{
    throw Error("set is read-only")
    }),Object.freeze(e),Object.getOwnPropertyNames(e).forEach((n=>{var i=e[n]
    ;"object"!=typeof i||Object.isFrozen(i)||t(i)})),e}
    e.exports=t,e.exports.default=t;var n=e.exports;class i{constructor(e){
    void 0===e.data&&(e.data={}),this.data=e.data,this.isMatchIgnored=!1}
    ignoreMatch(){this.isMatchIgnored=!0}}function r(e){
    return e.replace(/&/g,"&amp;").replace(/</g,"&lt;").replace(/>/g,"&gt;").replace(/"/g,"&quot;").replace(/'/g,"&#x27;")
    }function s(e,...t){const n=Object.create(null);for(const t in e)n[t]=e[t]
    ;return t.forEach((e=>{for(const t in e)n[t]=e[t]})),n}const o=e=>!!e.kind
    ;class a{constructor(e,t){
    this.buffer="",this.classPrefix=t.classPrefix,e.walk(this)}addText(e){
    this.buffer+=r(e)}openNode(e){if(!o(e))return;let t=e.kind
    ;t=e.sublanguage?"language-"+t:((e,{prefix:t})=>{if(e.includes(".")){
    const n=e.split(".")
    ;return[`${t}${n.shift()}`,...n.map(((e,t)=>`${e}${"_".repeat(t+1)}`))].join(" ")
    }return`${t}${e}`})(t,{prefix:this.classPrefix}),this.span(t)}closeNode(e){
    o(e)&&(this.buffer+="</span>")}value(){return this.buffer}span(e){
    this.buffer+=`<span class="${e}">`}}class l{constructor(){this.rootNode={
    children:[]},this.stack=[this.rootNode]}get top(){
    return this.stack[this.stack.length-1]}get root(){return this.rootNode}add(e){
    this.top.children.push(e)}openNode(e){const t={kind:e,children:[]}
    ;this.add(t),this.stack.push(t)}closeNode(){
    if(this.stack.length>1)return this.stack.pop()}closeAllNodes(){
    for(;this.closeNode(););}toJSON(){return JSON.stringify(this.rootNode,null,4)}
    walk(e){return this.constructor._walk(e,this.rootNode)}static _walk(e,t){
    return"string"==typeof t?e.addText(t):t.children&&(e.openNode(t),
    t.children.forEach((t=>this._walk(e,t))),e.closeNode(t)),e}static _collapse(e){
    "string"!=typeof e&&e.children&&(e.children.every((e=>"string"==typeof e))?e.children=[e.children.join("")]:e.children.forEach((e=>{
    l._collapse(e)})))}}class c extends l{constructor(e){super(),this.options=e}
    addKeyword(e,t){""!==e&&(this.openNode(t),this.addText(e),this.closeNode())}
    addText(e){""!==e&&this.add(e)}addSublanguage(e,t){const n=e.root
    ;n.kind=t,n.sublanguage=!0,this.add(n)}toHTML(){
    return new a(this,this.options).value()}finalize(){return!0}}function g(e){
    return e?"string"==typeof e?e:e.source:null}function d(...e){
    return e.map((e=>g(e))).join("")}function u(...e){return"("+((e=>{
    const t=e[e.length-1]
    ;return"object"==typeof t&&t.constructor===Object?(e.splice(e.length-1,1),t):{}
    })(e).capture?"":"?:")+e.map((e=>g(e))).join("|")+")"}function h(e){
    return RegExp(e.toString()+"|").exec("").length-1}
    const f=/\[(?:[^\\\]]|\\.)*\]|\(\??|\\([1-9][0-9]*)|\\./
    ;function p(e,{joinWith:t}){let n=0;return e.map((e=>{n+=1;const t=n
    ;let i=g(e),r="";for(;i.length>0;){const e=f.exec(i);if(!e){r+=i;break}
    r+=i.substring(0,e.index),
    i=i.substring(e.index+e[0].length),"\\"===e[0][0]&&e[1]?r+="\\"+(Number(e[1])+t):(r+=e[0],
    "("===e[0]&&n++)}return r})).map((e=>`(${e})`)).join(t)}
    const b="[a-zA-Z]\\w*",m="[a-zA-Z_]\\w*",E="\\b\\d+(\\.\\d+)?",x="(-?)(\\b0[xX][a-fA-F0-9]+|(\\b\\d+(\\.\\d*)?|\\.\\d+)([eE][-+]?\\d+)?)",y="\\b(0b[01]+)",w={
    begin:"\\\\[\\s\\S]",relevance:0},_={scope:"string",begin:"'",end:"'",
    illegal:"\\n",contains:[w]},v={scope:"string",begin:'"',end:'"',illegal:"\\n",
    contains:[w]},O=(e,t,n={})=>{const i=s({scope:"comment",begin:e,end:t,
    contains:[]},n);i.contains.push({scope:"doctag",
    begin:"[ ]*(?=(TODO|FIXME|NOTE|BUG|OPTIMIZE|HACK|XXX):)",
    end:/(TODO|FIXME|NOTE|BUG|OPTIMIZE|HACK|XXX):/,excludeBegin:!0,relevance:0})
    ;const r=u("I","a","is","so","us","to","at","if","in","it","on",/[A-Za-z]+['](d|ve|re|ll|t|s|n)/,/[A-Za-z]+[-][a-z]+/,/[A-Za-z][a-z]{2,}/)
    ;return i.contains.push({begin:d(/[ ]+/,"(",r,/[.]?[:]?([.][ ]|[ ])/,"){3}")}),i
    },k=O("//","$"),N=O("/\\*","\\*/"),S=O("#","$");var M=Object.freeze({
    __proto__:null,MATCH_NOTHING_RE:/\b\B/,IDENT_RE:b,UNDERSCORE_IDENT_RE:m,
    NUMBER_RE:E,C_NUMBER_RE:x,BINARY_NUMBER_RE:y,
    RE_STARTERS_RE:"!|!=|!==|%|%=|&|&&|&=|\\*|\\*=|\\+|\\+=|,|-|-=|/=|/|:|;|<<|<<=|<=|<|===|==|=|>>>=|>>=|>=|>>>|>>|>|\\?|\\[|\\{|\\(|\\^|\\^=|\\||\\|=|\\|\\||~",
    SHEBANG:(e={})=>{const t=/^#![ ]*\//
    ;return e.binary&&(e.begin=d(t,/.*\b/,e.binary,/\b.*/)),s({scope:"meta",begin:t,
    end:/$/,relevance:0,"on:begin":(e,t)=>{0!==e.index&&t.ignoreMatch()}},e)},
    BACKSLASH_ESCAPE:w,APOS_STRING_MODE:_,QUOTE_STRING_MODE:v,PHRASAL_WORDS_MODE:{
    begin:/\b(a|an|the|are|I'm|isn't|don't|doesn't|won't|but|just|should|pretty|simply|enough|gonna|going|wtf|so|such|will|you|your|they|like|more)\b/
    },COMMENT:O,C_LINE_COMMENT_MODE:k,C_BLOCK_COMMENT_MODE:N,HASH_COMMENT_MODE:S,
    NUMBER_MODE:{scope:"number",begin:E,relevance:0},C_NUMBER_MODE:{scope:"number",
    begin:x,relevance:0},BINARY_NUMBER_MODE:{scope:"number",begin:y,relevance:0},
    REGEXP_MODE:{begin:/(?=\/[^/\n]*\/)/,contains:[{scope:"regexp",begin:/\//,
    end:/\/[gimuy]*/,illegal:/\n/,contains:[w,{begin:/\[/,end:/\]/,relevance:0,
    contains:[w]}]}]},TITLE_MODE:{scope:"title",begin:b,relevance:0},
    UNDERSCORE_TITLE_MODE:{scope:"title",begin:m,relevance:0},METHOD_GUARD:{
    begin:"\\.\\s*[a-zA-Z_]\\w*",relevance:0},END_SAME_AS_BEGIN:e=>Object.assign(e,{
    "on:begin":(e,t)=>{t.data._beginMatch=e[1]},"on:end":(e,t)=>{
    t.data._beginMatch!==e[1]&&t.ignoreMatch()}})});function R(e,t){
    "."===e.input[e.index-1]&&t.ignoreMatch()}function j(e,t){
    void 0!==e.className&&(e.scope=e.className,delete e.className)}function A(e,t){
    t&&e.beginKeywords&&(e.begin="\\b("+e.beginKeywords.split(" ").join("|")+")(?!\\.)(?=\\b|\\s)",
    e.__beforeBegin=R,e.keywords=e.keywords||e.beginKeywords,delete e.beginKeywords,
    void 0===e.relevance&&(e.relevance=0))}function I(e,t){
    Array.isArray(e.illegal)&&(e.illegal=u(...e.illegal))}function B(e,t){
    if(e.match){
    if(e.begin||e.end)throw Error("begin & end are not supported with match")
    ;e.begin=e.match,delete e.match}}function T(e,t){
    void 0===e.relevance&&(e.relevance=1)}const L=(e,t)=>{if(!e.beforeMatch)return
    ;if(e.starts)throw Error("beforeMatch cannot be used with starts")
    ;const n=Object.assign({},e);Object.keys(e).forEach((t=>{delete e[t]
    })),e.keywords=n.keywords,
    e.begin=d(n.beforeMatch,d("(?=",n.begin,")")),e.starts={relevance:0,
    contains:[Object.assign(n,{endsParent:!0})]},e.relevance=0,delete n.beforeMatch
    },D=["of","and","for","in","not","or","if","then","parent","list","value"]
    ;function P(e,t,n="keyword"){const i=Object.create(null)
    ;return"string"==typeof e?r(n,e.split(" ")):Array.isArray(e)?r(n,e):Object.keys(e).forEach((n=>{
    Object.assign(i,P(e[n],t,n))})),i;function r(e,n){
    t&&(n=n.map((e=>e.toLowerCase()))),n.forEach((t=>{const n=t.split("|")
    ;i[n[0]]=[e,C(n[0],n[1])]}))}}function C(e,t){
    return t?Number(t):(e=>D.includes(e.toLowerCase()))(e)?0:1}const H={},$=e=>{
    console.error(e)},U=(e,...t)=>{console.log("WARN: "+e,...t)},z=(e,t)=>{
    H[`${e}/${t}`]||(console.log(`Deprecated as of ${e}. ${t}`),H[`${e}/${t}`]=!0)
    },K=Error();function W(e,t,{key:n}){let i=0;const r=e[n],s={},o={}
    ;for(let e=1;e<=t.length;e++)o[e+i]=r[e],s[e+i]=!0,i+=h(t[e-1])
    ;e[n]=o,e[n]._emit=s,e[n]._multi=!0}function X(e){(e=>{
    e.scope&&"object"==typeof e.scope&&null!==e.scope&&(e.beginScope=e.scope,
    delete e.scope)})(e),"string"==typeof e.beginScope&&(e.beginScope={
    _wrap:e.beginScope}),"string"==typeof e.endScope&&(e.endScope={_wrap:e.endScope
    }),(e=>{if(Array.isArray(e.begin)){
    if(e.skip||e.excludeBegin||e.returnBegin)throw $("skip, excludeBegin, returnBegin not compatible with beginScope: {}"),
    K
    ;if("object"!=typeof e.beginScope||null===e.beginScope)throw $("beginScope must be object"),
    K;W(e,e.begin,{key:"beginScope"}),e.begin=p(e.begin,{joinWith:""})}})(e),(e=>{
    if(Array.isArray(e.end)){
    if(e.skip||e.excludeEnd||e.returnEnd)throw $("skip, excludeEnd, returnEnd not compatible with endScope: {}"),
    K
    ;if("object"!=typeof e.endScope||null===e.endScope)throw $("endScope must be object"),
    K;W(e,e.end,{key:"endScope"}),e.end=p(e.end,{joinWith:""})}})(e)}function G(e){
    function t(t,n){return RegExp(g(t),"m"+(e.case_insensitive?"i":"")+(n?"g":""))}
    class n{constructor(){
    this.matchIndexes={},this.regexes=[],this.matchAt=1,this.position=0}
    addRule(e,t){
    t.position=this.position++,this.matchIndexes[this.matchAt]=t,this.regexes.push([t,e]),
    this.matchAt+=h(e)+1}compile(){0===this.regexes.length&&(this.exec=()=>null)
    ;const e=this.regexes.map((e=>e[1]));this.matcherRe=t(p(e,{joinWith:"|"
    }),!0),this.lastIndex=0}exec(e){this.matcherRe.lastIndex=this.lastIndex
    ;const t=this.matcherRe.exec(e);if(!t)return null
    ;const n=t.findIndex(((e,t)=>t>0&&void 0!==e)),i=this.matchIndexes[n]
    ;return t.splice(0,n),Object.assign(t,i)}}class i{constructor(){
    this.rules=[],this.multiRegexes=[],
    this.count=0,this.lastIndex=0,this.regexIndex=0}getMatcher(e){
    if(this.multiRegexes[e])return this.multiRegexes[e];const t=new n
    ;return this.rules.slice(e).forEach((([e,n])=>t.addRule(e,n))),
    t.compile(),this.multiRegexes[e]=t,t}resumingScanAtSamePosition(){
    return 0!==this.regexIndex}considerAll(){this.regexIndex=0}addRule(e,t){
    this.rules.push([e,t]),"begin"===t.type&&this.count++}exec(e){
    const t=this.getMatcher(this.regexIndex);t.lastIndex=this.lastIndex
    ;let n=t.exec(e)
    ;if(this.resumingScanAtSamePosition())if(n&&n.index===this.lastIndex);else{
    const t=this.getMatcher(0);t.lastIndex=this.lastIndex+1,n=t.exec(e)}
    return n&&(this.regexIndex+=n.position+1,
    this.regexIndex===this.count&&this.considerAll()),n}}
    if(e.compilerExtensions||(e.compilerExtensions=[]),
    e.contains&&e.contains.includes("self"))throw Error("ERR: contains `self` is not supported at the top-level of a language.  See documentation.")
    ;return e.classNameAliases=s(e.classNameAliases||{}),function n(r,o){const a=r
    ;if(r.isCompiled)return a
    ;[j,B,X,L].forEach((e=>e(r,o))),e.compilerExtensions.forEach((e=>e(r,o))),
    r.__beforeBegin=null,[A,I,T].forEach((e=>e(r,o))),r.isCompiled=!0;let l=null
    ;return"object"==typeof r.keywords&&r.keywords.$pattern&&(r.keywords=Object.assign({},r.keywords),
    l=r.keywords.$pattern,
    delete r.keywords.$pattern),l=l||/\w+/,r.keywords&&(r.keywords=P(r.keywords,e.case_insensitive)),
    a.keywordPatternRe=t(l,!0),
    o&&(r.begin||(r.begin=/\B|\b/),a.beginRe=t(r.begin),r.end||r.endsWithParent||(r.end=/\B|\b/),
    r.end&&(a.endRe=t(r.end)),
    a.terminatorEnd=g(r.end)||"",r.endsWithParent&&o.terminatorEnd&&(a.terminatorEnd+=(r.end?"|":"")+o.terminatorEnd)),
    r.illegal&&(a.illegalRe=t(r.illegal)),
    r.contains||(r.contains=[]),r.contains=[].concat(...r.contains.map((e=>(e=>(e.variants&&!e.cachedVariants&&(e.cachedVariants=e.variants.map((t=>s(e,{
    variants:null},t)))),e.cachedVariants?e.cachedVariants:Z(e)?s(e,{
    starts:e.starts?s(e.starts):null
    }):Object.isFrozen(e)?s(e):e))("self"===e?r:e)))),r.contains.forEach((e=>{n(e,a)
    })),r.starts&&n(r.starts,o),a.matcher=(e=>{const t=new i
    ;return e.contains.forEach((e=>t.addRule(e.begin,{rule:e,type:"begin"
    }))),e.terminatorEnd&&t.addRule(e.terminatorEnd,{type:"end"
    }),e.illegal&&t.addRule(e.illegal,{type:"illegal"}),t})(a),a}(e)}function Z(e){
    return!!e&&(e.endsWithParent||Z(e.starts))}const F=r,V=s,q=Symbol("nomatch")
    ;var J=(e=>{const t=Object.create(null),r=Object.create(null),s=[];let o=!0
    ;const a="Could not find the language '{}', did you forget to load/include a language module?",l={
    disableAutodetect:!0,name:"Plain text",contains:[]};let g={
    ignoreUnescapedHTML:!1,noHighlightRe:/^(no-?highlight)$/i,
    languageDetectRe:/\blang(?:uage)?-([\w-]+)\b/i,classPrefix:"hljs-",
    cssSelector:"pre code",languages:null,__emitter:c};function d(e){
    return g.noHighlightRe.test(e)}function u(e,t,n,i){let r="",s=""
    ;"object"==typeof t?(r=e,
    n=t.ignoreIllegals,s=t.language,i=void 0):(z("10.7.0","highlight(lang, code, ...args) has been deprecated."),
    z("10.7.0","Please use highlight(code, options) instead.\nhttps://github.com/highlightjs/highlight.js/issues/2277"),
    s=e,r=t),void 0===n&&(n=!0);const o={code:r,language:s};w("before:highlight",o)
    ;const a=o.result?o.result:h(o.language,o.code,n,i)
    ;return a.code=o.code,w("after:highlight",a),a}function h(e,n,r,s){
    const l=Object.create(null);function c(){if(!k.keywords)return void S.addText(M)
    ;let e=0;k.keywordPatternRe.lastIndex=0;let t=k.keywordPatternRe.exec(M),n=""
    ;for(;t;){n+=M.substring(e,t.index)
    ;const r=_.case_insensitive?t[0].toLowerCase():t[0],s=(i=r,k.keywords[i]);if(s){
    const[e,i]=s
    ;if(S.addText(n),n="",l[r]=(l[r]||0)+1,l[r]<=7&&(R+=i),e.startsWith("_"))n+=t[0];else{
    const n=_.classNameAliases[e]||e;S.addKeyword(t[0],n)}}else n+=t[0]
    ;e=k.keywordPatternRe.lastIndex,t=k.keywordPatternRe.exec(M)}var i
    ;n+=M.substr(e),S.addText(n)}function d(){null!=k.subLanguage?(()=>{
    if(""===M)return;let e=null;if("string"==typeof k.subLanguage){
    if(!t[k.subLanguage])return void S.addText(M)
    ;e=h(k.subLanguage,M,!0,N[k.subLanguage]),N[k.subLanguage]=e._top
    }else e=f(M,k.subLanguage.length?k.subLanguage:null)
    ;k.relevance>0&&(R+=e.relevance),S.addSublanguage(e._emitter,e.language)
    })():c(),M=""}function u(e,t){let n=1;for(;void 0!==t[n];){if(!e._emit[n]){n++
    ;continue}const i=_.classNameAliases[e[n]]||e[n],r=t[n]
    ;i?S.addKeyword(r,i):(M=r,c(),M=""),n++}}function p(e,t){
    return e.scope&&"string"==typeof e.scope&&S.openNode(_.classNameAliases[e.scope]||e.scope),
    e.beginScope&&(e.beginScope._wrap?(S.addKeyword(M,_.classNameAliases[e.beginScope._wrap]||e.beginScope._wrap),
    M=""):e.beginScope._multi&&(u(e.beginScope,t),M="")),k=Object.create(e,{parent:{
    value:k}}),k}function b(e,t,n){let r=((e,t)=>{const n=e&&e.exec(t)
    ;return n&&0===n.index})(e.endRe,n);if(r){if(e["on:end"]){const n=new i(e)
    ;e["on:end"](t,n),n.isMatchIgnored&&(r=!1)}if(r){
    for(;e.endsParent&&e.parent;)e=e.parent;return e}}
    if(e.endsWithParent)return b(e.parent,t,n)}function m(e){
    return 0===k.matcher.regexIndex?(M+=e[0],1):(I=!0,0)}function x(e){
    const t=e[0],i=n.substr(e.index),r=b(k,e,i);if(!r)return q;const s=k
    ;k.endScope&&k.endScope._wrap?(d(),
    S.addKeyword(t,k.endScope._wrap)):k.endScope&&k.endScope._multi?(d(),
    u(k.endScope,e)):s.skip?M+=t:(s.returnEnd||s.excludeEnd||(M+=t),
    d(),s.excludeEnd&&(M=t));do{
    k.scope&&!k.isMultiClass&&S.closeNode(),k.skip||k.subLanguage||(R+=k.relevance),
    k=k.parent}while(k!==r.parent)
    ;return r.starts&&p(r.starts,e),s.returnEnd?0:t.length}let y={};function w(t,s){
    const a=s&&s[0];if(M+=t,null==a)return d(),0
    ;if("begin"===y.type&&"end"===s.type&&y.index===s.index&&""===a){
    if(M+=n.slice(s.index,s.index+1),!o){const t=Error(`0 width match regex (${e})`)
    ;throw t.languageName=e,t.badRule=y.rule,t}return 1}
    if(y=s,"begin"===s.type)return(e=>{
    const t=e[0],n=e.rule,r=new i(n),s=[n.__beforeBegin,n["on:begin"]]
    ;for(const n of s)if(n&&(n(e,r),r.isMatchIgnored))return m(t)
    ;return n.skip?M+=t:(n.excludeBegin&&(M+=t),
    d(),n.returnBegin||n.excludeBegin||(M=t)),p(n,e),n.returnBegin?0:t.length})(s)
    ;if("illegal"===s.type&&!r){
    const e=Error('Illegal lexeme "'+a+'" for mode "'+(k.scope||"<unnamed>")+'"')
    ;throw e.mode=k,e}if("end"===s.type){const e=x(s);if(e!==q)return e}
    if("illegal"===s.type&&""===a)return 1
    ;if(A>1e5&&A>3*s.index)throw Error("potential infinite loop, way more iterations than matches")
    ;return M+=a,a.length}const _=E(e)
    ;if(!_)throw $(a.replace("{}",e)),Error('Unknown language: "'+e+'"')
    ;const v=G(_);let O="",k=s||v;const N={},S=new g.__emitter(g);(()=>{const e=[]
    ;for(let t=k;t!==_;t=t.parent)t.scope&&e.unshift(t.scope)
    ;e.forEach((e=>S.openNode(e)))})();let M="",R=0,j=0,A=0,I=!1;try{
    for(k.matcher.considerAll();;){
    A++,I?I=!1:k.matcher.considerAll(),k.matcher.lastIndex=j
    ;const e=k.matcher.exec(n);if(!e)break;const t=w(n.substring(j,e.index),e)
    ;j=e.index+t}return w(n.substr(j)),S.closeAllNodes(),S.finalize(),O=S.toHTML(),{
    language:e,value:O,relevance:R,illegal:!1,_emitter:S,_top:k}}catch(t){
    if(t.message&&t.message.includes("Illegal"))return{language:e,value:F(n),
    illegal:!0,relevance:0,_illegalBy:{message:t.message,index:j,
    context:n.slice(j-100,j+100),mode:t.mode,resultSoFar:O},_emitter:S};if(o)return{
    language:e,value:F(n),illegal:!1,relevance:0,errorRaised:t,_emitter:S,_top:k}
    ;throw t}}function f(e,n){n=n||g.languages||Object.keys(t);const i=(e=>{
    const t={value:F(e),illegal:!1,relevance:0,_top:l,_emitter:new g.__emitter(g)}
    ;return t._emitter.addText(e),t})(e),r=n.filter(E).filter(y).map((t=>h(t,e,!1)))
    ;r.unshift(i);const s=r.sort(((e,t)=>{
    if(e.relevance!==t.relevance)return t.relevance-e.relevance
    ;if(e.language&&t.language){if(E(e.language).supersetOf===t.language)return 1
    ;if(E(t.language).supersetOf===e.language)return-1}return 0})),[o,a]=s,c=o
    ;return c.secondBest=a,c}function p(e){let t=null;const n=(e=>{
    let t=e.className+" ";t+=e.parentNode?e.parentNode.className:""
    ;const n=g.languageDetectRe.exec(t);if(n){const t=E(n[1])
    ;return t||(U(a.replace("{}",n[1])),
    U("Falling back to no-highlight mode for this block.",e)),t?n[1]:"no-highlight"}
    return t.split(/\s+/).find((e=>d(e)||E(e)))})(e);if(d(n))return
    ;w("before:highlightElement",{el:e,language:n
    }),!g.ignoreUnescapedHTML&&e.children.length>0&&(console.warn("One of your code blocks includes unescaped HTML. This is a potentially serious security risk."),
    console.warn("https://github.com/highlightjs/highlight.js/issues/2886"),
    console.warn(e)),t=e;const i=t.textContent,s=n?u(i,{language:n,ignoreIllegals:!0
    }):f(i);e.innerHTML=s.value,((e,t,n)=>{const i=t&&r[t]||n
    ;e.classList.add("hljs"),e.classList.add("language-"+i)
    })(e,n,s.language),e.result={language:s.language,re:s.relevance,
    relevance:s.relevance},s.secondBest&&(e.secondBest={
    language:s.secondBest.language,relevance:s.secondBest.relevance
    }),w("after:highlightElement",{el:e,result:s,text:i})}let b=!1;function m(){
    "loading"!==document.readyState?document.querySelectorAll(g.cssSelector).forEach(p):b=!0
    }function E(e){return e=(e||"").toLowerCase(),t[e]||t[r[e]]}
    function x(e,{languageName:t}){"string"==typeof e&&(e=[e]),e.forEach((e=>{
    r[e.toLowerCase()]=t}))}function y(e){const t=E(e)
    ;return t&&!t.disableAutodetect}function w(e,t){const n=e;s.forEach((e=>{
    e[n]&&e[n](t)}))}
    "undefined"!=typeof window&&window.addEventListener&&window.addEventListener("DOMContentLoaded",(()=>{
    b&&m()}),!1),Object.assign(e,{highlight:u,highlightAuto:f,highlightAll:m,
    highlightElement:p,
    highlightBlock:e=>(z("10.7.0","highlightBlock will be removed entirely in v12.0"),
    z("10.7.0","Please use highlightElement now."),p(e)),configure:e=>{g=V(g,e)},
    initHighlighting:()=>{
    m(),z("10.6.0","initHighlighting() deprecated.  Use highlightAll() now.")},
    initHighlightingOnLoad:()=>{
    m(),z("10.6.0","initHighlightingOnLoad() deprecated.  Use highlightAll() now.")
    },registerLanguage:(n,i)=>{let r=null;try{r=i(e)}catch(e){
    if($("Language definition for '{}' could not be registered.".replace("{}",n)),
    !o)throw e;$(e),r=l}
    r.name||(r.name=n),t[n]=r,r.rawDefinition=i.bind(null,e),r.aliases&&x(r.aliases,{
    languageName:n})},unregisterLanguage:e=>{delete t[e]
    ;for(const t of Object.keys(r))r[t]===e&&delete r[t]},
    listLanguages:()=>Object.keys(t),getLanguage:E,registerAliases:x,
    autoDetection:y,inherit:V,addPlugin:e=>{(e=>{
    e["before:highlightBlock"]&&!e["before:highlightElement"]&&(e["before:highlightElement"]=t=>{
    e["before:highlightBlock"](Object.assign({block:t.el},t))
    }),e["after:highlightBlock"]&&!e["after:highlightElement"]&&(e["after:highlightElement"]=t=>{
    e["after:highlightBlock"](Object.assign({block:t.el},t))})})(e),s.push(e)}
    }),e.debugMode=()=>{o=!1},e.safeMode=()=>{o=!0},e.versionString="11.0.1"
    ;for(const e in M)"object"==typeof M[e]&&n(M[e]);return Object.assign(e,M),e
    })({}),Y=Object.freeze({__proto__:null});const Q=J
    ;for(const e of Object.keys(Y)){const t=e.replace("grmr_","")
    ;Q.registerLanguage(t,Y[e])}return Q}()
    ;"object"==typeof exports&&"undefined"!=typeof module&&(module.exports=hljs);hljs.registerLanguage("scala",(()=>{"use strict";return e=>{const n={
    className:"subst",variants:[{begin:"\\$[A-Za-z0-9_]+"},{begin:/\$\{/,end:/\}/}]
    },a={className:"string",variants:[{begin:'"""',end:'"""'},{begin:'"',end:'"',
    illegal:"\\n",contains:[e.BACKSLASH_ESCAPE]},{begin:'[a-z]+"',end:'"',
    illegal:"\\n",contains:[e.BACKSLASH_ESCAPE,n]},{className:"string",
    begin:'[a-z]+"""',end:'"""',contains:[n],relevance:10}]},s={className:"type",
    begin:"\\b[A-Z][A-Za-z0-9_]*",relevance:0},t={className:"title",
    begin:/[^0-9\n\t "'(),.`{}\[\]:;][^\n\t "'(),.`{}\[\]:;]+|[^0-9\n\t "'(),.`{}\[\]:;=]/,
    relevance:0},i={className:"class",beginKeywords:"class object trait type",
    end:/[:={\[\n;]/,excludeEnd:!0,
    contains:[e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE,{
    beginKeywords:"extends with",relevance:10},{begin:/\[/,end:/\]/,excludeBegin:!0,
    excludeEnd:!0,relevance:0,contains:[s]},{className:"params",begin:/\(/,end:/\)/,
    excludeBegin:!0,excludeEnd:!0,relevance:0,contains:[s]},t]},l={
    className:"function",beginKeywords:"def",end:/[:={\[(\n;]/,excludeEnd:!0,
    contains:[t]};return{name:"Scala",keywords:{literal:"true false null",
    keyword:"type yield lazy override def with val var sealed abstract private trait object if forSome for while throw finally protected extends import final return else break new catch super class case package default try this match continue throws implicit"
    },contains:[e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE,a,{className:"symbol",
    begin:"'\\w[\\w\\d_]*(?!')"},s,l,i,e.C_NUMBER_MODE,{className:"meta",
    begin:"@[A-Za-z]+"}]}}})());hljs.registerLanguage("typescript",(()=>{"use strict"
    ;const e="[A-Za-z$_][0-9A-Za-z$_]*",n=["as","in","of","if","for","while","finally","var","new","function","do","return","void","else","break","catch","instanceof","with","throw","case","default","try","switch","continue","typeof","delete","let","yield","const","class","debugger","async","await","static","import","from","export","extends"],a=["true","false","null","undefined","NaN","Infinity"],t=["Intl","DataView","Number","Math","Date","String","RegExp","Object","Function","Boolean","Error","Symbol","Set","Map","WeakSet","WeakMap","Proxy","Reflect","JSON","Promise","Float64Array","Int16Array","Int32Array","Int8Array","Uint16Array","Uint32Array","Float32Array","Array","Uint8Array","Uint8ClampedArray","ArrayBuffer","BigInt64Array","BigUint64Array","BigInt"],s=["EvalError","InternalError","RangeError","ReferenceError","SyntaxError","TypeError","URIError"],r=["setInterval","setTimeout","clearInterval","clearTimeout","require","exports","eval","isFinite","isNaN","parseFloat","parseInt","decodeURI","decodeURIComponent","encodeURI","encodeURIComponent","escape","unescape"],c=["arguments","this","super","console","window","document","localStorage","module","global"],i=[].concat(r,t,s)
    ;function o(e){return l("(?=",e,")")}function l(...e){return e.map((e=>{
    return(n=e)?"string"==typeof n?n:n.source:null;var n})).join("")}return b=>{
    const d={$pattern:e,
    keyword:n.concat(["type","namespace","typedef","interface","public","private","protected","implements","declare","abstract","readonly"]),
    literal:a,
    built_in:i.concat(["any","void","number","boolean","string","object","never","enum"]),
    "variable.language":c},g={className:"meta",begin:"@[A-Za-z$_][0-9A-Za-z$_]*"
    },u=(e,n,a)=>{const t=e.contains.findIndex((e=>e.label===n))
    ;if(-1===t)throw Error("can not find mode to replace");e.contains.splice(t,1,a)
    },m=function(b){const d=e,g={begin:/<[A-Za-z0-9\\._:-]+/,
    end:/\/[A-Za-z0-9\\._:-]+>|\/>/,isTrulyOpeningTag:(e,n)=>{
    const a=e[0].length+e.index,t=e.input[a];"<"!==t?">"===t&&(((e,{after:n})=>{
    const a="</"+e[0].slice(1);return-1!==e.input.indexOf(a,n)})(e,{after:a
    })||n.ignoreMatch()):n.ignoreMatch()}},u={$pattern:e,keyword:n,literal:a,
    built_in:i,"variable.language":c
    },m="\\.([0-9](_?[0-9])*)",E="0|[1-9](_?[0-9])*|0[0-7]*[89][0-9]*",y={
    className:"number",variants:[{
    begin:`(\\b(${E})((${m})|\\.)?|(${m}))[eE][+-]?([0-9](_?[0-9])*)\\b`},{
    begin:`\\b(${E})\\b((${m})\\b|\\.)?|(${m})\\b`},{
    begin:"\\b(0|[1-9](_?[0-9])*)n\\b"},{
    begin:"\\b0[xX][0-9a-fA-F](_?[0-9a-fA-F])*n?\\b"},{
    begin:"\\b0[bB][0-1](_?[0-1])*n?\\b"},{begin:"\\b0[oO][0-7](_?[0-7])*n?\\b"},{
    begin:"\\b0[0-7]+n?\\b"}],relevance:0},f={className:"subst",begin:"\\$\\{",
    end:"\\}",keywords:u,contains:[]},p={begin:"html`",end:"",starts:{end:"`",
    returnEnd:!1,contains:[b.BACKSLASH_ESCAPE,f],subLanguage:"xml"}},_={
    begin:"css`",end:"",starts:{end:"`",returnEnd:!1,
    contains:[b.BACKSLASH_ESCAPE,f],subLanguage:"css"}},N={className:"string",
    begin:"`",end:"`",contains:[b.BACKSLASH_ESCAPE,f]},A={className:"comment",
    variants:[b.COMMENT(/\/\*\*(?!\/)/,"\\*/",{relevance:0,contains:[{
    begin:"(?=@[A-Za-z]+)",relevance:0,contains:[{className:"doctag",
    begin:"@[A-Za-z]+"},{className:"type",begin:"\\{",end:"\\}",excludeEnd:!0,
    excludeBegin:!0,relevance:0},{className:"variable",begin:d+"(?=\\s*(-)|$)",
    endsParent:!0,relevance:0},{begin:/(?=[^\n])\s/,relevance:0}]}]
    }),b.C_BLOCK_COMMENT_MODE,b.C_LINE_COMMENT_MODE]
    },v=[b.APOS_STRING_MODE,b.QUOTE_STRING_MODE,p,_,N,y,b.REGEXP_MODE]
    ;f.contains=v.concat({begin:/\{/,end:/\}/,keywords:u,contains:["self"].concat(v)
    });const h=[].concat(A,f.contains),w=h.concat([{begin:/\(/,end:/\)/,keywords:u,
    contains:["self"].concat(h)}]),S={className:"params",begin:/\(/,end:/\)/,
    excludeBegin:!0,excludeEnd:!0,keywords:u,contains:w},O={variants:[{
    match:[/class/,/\s+/,d],scope:{1:"keyword",3:"title.class"}},{
    match:[/extends/,/\s+/,l(d,"(",l(/\./,d),")*")],scope:{1:"keyword",
    3:"title.class.inherited"}}]},R={relevance:0,
    match:/\b[A-Z][a-z]+([A-Z][a-z]+)*/,className:"title.class",keywords:{
    _:[...t,...s]}},I={variants:[{match:[/function/,/\s+/,d,/(?=\s*\()/]},{
    match:[/function/,/\s*(?=\()/]}],className:{1:"keyword",3:"title.function"},
    label:"func.def",contains:[S],illegal:/%/},x={
    match:l(/\b/,(T=[...r,"super"],l("(?!",T.join("|"),")")),d,o(/\(/)),
    className:"title.function",relevance:0};var T;const k={
    begin:l(/\./,o(l(d,/(?![0-9A-Za-z$_(])/))),end:d,excludeBegin:!0,
    keywords:"prototype",className:"property",relevance:0},M={
    match:[/get|set/,/\s+/,d,/(?=\()/],className:{1:"keyword",3:"title.function"},
    contains:[{begin:/\(\)/},S]
    },C="(\\([^()]*(\\([^()]*(\\([^()]*\\)[^()]*)*\\)[^()]*)*\\)|"+b.UNDERSCORE_IDENT_RE+")\\s*=>",B={
    match:[/const|var|let/,/\s+/,d,/\s*/,/=\s*/,o(C)],className:{1:"keyword",
    3:"title.function"},contains:[S]};return{name:"Javascript",
    aliases:["js","jsx","mjs","cjs"],keywords:u,exports:{PARAMS_CONTAINS:w},
    illegal:/#(?![$_A-z])/,contains:[b.SHEBANG({label:"shebang",binary:"node",
    relevance:5}),{label:"use_strict",className:"meta",relevance:10,
    begin:/^\s*['"]use (strict|asm)['"]/
    },b.APOS_STRING_MODE,b.QUOTE_STRING_MODE,p,_,N,A,y,R,{className:"attr",
    begin:d+o(":"),relevance:0},B,{
    begin:"("+b.RE_STARTERS_RE+"|\\b(case|return|throw)\\b)\\s*",
    keywords:"return throw case",relevance:0,contains:[A,b.REGEXP_MODE,{
    className:"function",begin:C,returnBegin:!0,end:"\\s*=>",contains:[{
    className:"params",variants:[{begin:b.UNDERSCORE_IDENT_RE,relevance:0},{
    className:null,begin:/\(\s*\)/,skip:!0},{begin:/\(/,end:/\)/,excludeBegin:!0,
    excludeEnd:!0,keywords:u,contains:w}]}]},{begin:/,/,relevance:0},{match:/\s+/,
    relevance:0},{variants:[{begin:"<>",end:"</>"},{begin:g.begin,
    "on:begin":g.isTrulyOpeningTag,end:g.end}],subLanguage:"xml",contains:[{
    begin:g.begin,end:g.end,skip:!0,contains:["self"]}]}]},I,{
    beginKeywords:"while if switch catch for"},{
    begin:"\\b(?!function)"+b.UNDERSCORE_IDENT_RE+"\\([^()]*(\\([^()]*(\\([^()]*\\)[^()]*)*\\)[^()]*)*\\)\\s*\\{",
    returnBegin:!0,label:"func.def",contains:[S,b.inherit(b.TITLE_MODE,{begin:d,
    className:"title.function"})]},{match:/\.\.\./,relevance:0},k,{match:"\\$"+d,
    relevance:0},{match:[/\bconstructor(?=\s*\()/],className:{1:"title.function"},
    contains:[S]},x,{relevance:0,match:/\b[A-Z][A-Z_]+\b/,
    className:"variable.constant"},O,M,{match:/\$[(.]/}]}}(b)
    ;return Object.assign(m.keywords,d),
    m.exports.PARAMS_CONTAINS.push(g),m.contains=m.contains.concat([g,{
    beginKeywords:"namespace",end:/\{/,excludeEnd:!0},{beginKeywords:"interface",
    end:/\{/,excludeEnd:!0,keywords:"interface extends"
    }]),u(m,"shebang",b.SHEBANG()),u(m,"use_strict",{className:"meta",relevance:10,
    begin:/^\s*['"]use strict['"]/
    }),m.contains.find((e=>"func.def"===e.label)).relevance=0,Object.assign(m,{
    name:"TypeScript",aliases:["ts","tsx"]}),m}})());hljs.registerLanguage("objectivec",(()=>{"use strict";return e=>{
    const n=/[a-zA-Z@][a-zA-Z0-9_]*/,_={$pattern:n,
    keyword:["@interface","@class","@protocol","@implementation"]};return{
    name:"Objective-C",aliases:["mm","objc","obj-c","obj-c++","objective-c++"],
    keywords:{$pattern:n,
    keyword:["int","float","while","char","export","sizeof","typedef","const","struct","for","union","unsigned","long","volatile","static","bool","mutable","if","do","return","goto","void","enum","else","break","extern","asm","case","short","default","double","register","explicit","signed","typename","this","switch","continue","wchar_t","inline","readonly","assign","readwrite","self","@synchronized","id","typeof","nonatomic","super","unichar","IBOutlet","IBAction","strong","weak","copy","in","out","inout","bycopy","byref","oneway","__strong","__weak","__block","__autoreleasing","@private","@protected","@public","@try","@property","@end","@throw","@catch","@finally","@autoreleasepool","@synthesize","@dynamic","@selector","@optional","@required","@encode","@package","@import","@defs","@compatibility_alias","__bridge","__bridge_transfer","__bridge_retained","__bridge_retain","__covariant","__contravariant","__kindof","_Nonnull","_Nullable","_Null_unspecified","__FUNCTION__","__PRETTY_FUNCTION__","__attribute__","getter","setter","retain","unsafe_unretained","nonnull","nullable","null_unspecified","null_resettable","class","instancetype","NS_DESIGNATED_INITIALIZER","NS_UNAVAILABLE","NS_REQUIRES_SUPER","NS_RETURNS_INNER_POINTER","NS_INLINE","NS_AVAILABLE","NS_DEPRECATED","NS_ENUM","NS_OPTIONS","NS_SWIFT_UNAVAILABLE","NS_ASSUME_NONNULL_BEGIN","NS_ASSUME_NONNULL_END","NS_REFINED_FOR_SWIFT","NS_SWIFT_NAME","NS_SWIFT_NOTHROW","NS_DURING","NS_HANDLER","NS_ENDHANDLER","NS_VALUERETURN","NS_VOIDRETURN"],
    literal:["false","true","FALSE","TRUE","nil","YES","NO","NULL"],
    built_in:["BOOL","dispatch_once_t","dispatch_queue_t","dispatch_sync","dispatch_async","dispatch_once"]
    },illegal:"</",contains:[{className:"built_in",
    begin:"\\b(AV|CA|CF|CG|CI|CL|CM|CN|CT|MK|MP|MTK|MTL|NS|SCN|SK|UI|WK|XC)\\w+"
    },e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE,e.C_NUMBER_MODE,e.QUOTE_STRING_MODE,e.APOS_STRING_MODE,{
    className:"string",variants:[{begin:'@"',end:'"',illegal:"\\n",
    contains:[e.BACKSLASH_ESCAPE]}]},{className:"meta",begin:/#\s*[a-z]+\b/,end:/$/,
    keywords:{
    keyword:"if else elif endif define undef warning error line pragma ifdef ifndef include"
    },contains:[{begin:/\\\n/,relevance:0},e.inherit(e.QUOTE_STRING_MODE,{
    className:"string"}),{className:"string",begin:/<.*?>/,end:/$/,illegal:"\\n"
    },e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE]},{className:"class",
    begin:"("+_.keyword.join("|")+")\\b",end:/(\{|$)/,excludeEnd:!0,keywords:_,
    contains:[e.UNDERSCORE_TITLE_MODE]},{begin:"\\."+e.UNDERSCORE_IDENT_RE,
    relevance:0}]}}})());hljs.registerLanguage("kotlin",(()=>{"use strict"
    ;var e="\\.([0-9](_*[0-9])*)",n="[0-9a-fA-F](_*[0-9a-fA-F])*",a={
    className:"number",variants:[{
    begin:`(\\b([0-9](_*[0-9])*)((${e})|\\.)?|(${e}))[eE][+-]?([0-9](_*[0-9])*)[fFdD]?\\b`
    },{begin:`\\b([0-9](_*[0-9])*)((${e})[fFdD]?\\b|\\.([fFdD]\\b)?)`},{
    begin:`(${e})[fFdD]?\\b`},{begin:"\\b([0-9](_*[0-9])*)[fFdD]\\b"},{
    begin:`\\b0[xX]((${n})\\.?|(${n})?\\.(${n}))[pP][+-]?([0-9](_*[0-9])*)[fFdD]?\\b`
    },{begin:"\\b(0|[1-9](_*[0-9])*)[lL]?\\b"},{begin:`\\b0[xX](${n})[lL]?\\b`},{
    begin:"\\b0(_*[0-7])*[lL]?\\b"},{begin:"\\b0[bB][01](_*[01])*[lL]?\\b"}],
    relevance:0};return e=>{const n={
    keyword:"abstract as val var vararg get set class object open private protected public noinline crossinline dynamic final enum if else do while for when throw try catch finally import package is in fun override companion reified inline lateinit init interface annotation data sealed internal infix operator out by constructor super tailrec where const inner suspend typealias external expect actual",
    built_in:"Byte Short Char Int Long Boolean Float Double Void Unit Nothing",
    literal:"true false null"},i={className:"symbol",begin:e.UNDERSCORE_IDENT_RE+"@"
    },s={className:"subst",begin:/\$\{/,end:/\}/,contains:[e.C_NUMBER_MODE]},t={
    className:"variable",begin:"\\$"+e.UNDERSCORE_IDENT_RE},r={className:"string",
    variants:[{begin:'"""',end:'"""(?=[^"])',contains:[t,s]},{begin:"'",end:"'",
    illegal:/\n/,contains:[e.BACKSLASH_ESCAPE]},{begin:'"',end:'"',illegal:/\n/,
    contains:[e.BACKSLASH_ESCAPE,t,s]}]};s.contains.push(r);const l={
    className:"meta",
    begin:"@(?:file|property|field|get|set|receiver|param|setparam|delegate)\\s*:(?:\\s*"+e.UNDERSCORE_IDENT_RE+")?"
    },c={className:"meta",begin:"@"+e.UNDERSCORE_IDENT_RE,contains:[{begin:/\(/,
    end:/\)/,contains:[e.inherit(r,{className:"string"})]}]
    },o=a,b=e.COMMENT("/\\*","\\*/",{contains:[e.C_BLOCK_COMMENT_MODE]}),E={
    variants:[{className:"type",begin:e.UNDERSCORE_IDENT_RE},{begin:/\(/,end:/\)/,
    contains:[]}]},d=E;return d.variants[1].contains=[E],E.variants[1].contains=[d],
    {name:"Kotlin",aliases:["kt","kts"],keywords:n,
    contains:[e.COMMENT("/\\*\\*","\\*/",{relevance:0,contains:[{className:"doctag",
    begin:"@[A-Za-z]+"}]}),e.C_LINE_COMMENT_MODE,b,{className:"keyword",
    begin:/\b(break|continue|return|this)\b/,starts:{contains:[{className:"symbol",
    begin:/@\w+/}]}},i,l,c,{className:"function",beginKeywords:"fun",end:"[(]|$",
    returnBegin:!0,excludeEnd:!0,keywords:n,relevance:5,contains:[{
    begin:e.UNDERSCORE_IDENT_RE+"\\s*\\(",returnBegin:!0,relevance:0,
    contains:[e.UNDERSCORE_TITLE_MODE]},{className:"type",begin:/</,end:/>/,
    keywords:"reified",relevance:0},{className:"params",begin:/\(/,end:/\)/,
    endsParent:!0,keywords:n,relevance:0,contains:[{begin:/:/,end:/[=,\/]/,
    endsWithParent:!0,contains:[E,e.C_LINE_COMMENT_MODE,b],relevance:0
    },e.C_LINE_COMMENT_MODE,b,l,c,r,e.C_NUMBER_MODE]},b]},{className:"class",
    beginKeywords:"class interface trait",end:/[:\{(]|$/,excludeEnd:!0,
    illegal:"extends implements",contains:[{
    beginKeywords:"public protected internal private constructor"
    },e.UNDERSCORE_TITLE_MODE,{className:"type",begin:/</,end:/>/,excludeBegin:!0,
    excludeEnd:!0,relevance:0},{className:"type",begin:/[,:]\s*/,end:/[<\(,]|$/,
    excludeBegin:!0,returnEnd:!0},l,c]},r,{className:"meta",begin:"^#!/usr/bin/env",
    end:"$",illegal:"\n"},o]}}})());hljs.registerLanguage("go",(()=>{"use strict";return e=>{const n={
    keyword:["break","default","func","interface","select","case","map","struct","chan","else","goto","package","switch","const","fallthrough","if","range","type","continue","for","import","return","var","go","defer","bool","byte","complex64","complex128","float32","float64","int8","int16","int32","int64","string","uint8","uint16","uint32","uint64","int","uint","uintptr","rune"],
    literal:["true","false","iota","nil"],
    built_in:["append","cap","close","complex","copy","imag","len","make","new","panic","print","println","real","recover","delete"]
    };return{name:"Go",aliases:["golang"],keywords:n,illegal:"</",
    contains:[e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE,{className:"string",
    variants:[e.QUOTE_STRING_MODE,e.APOS_STRING_MODE,{begin:"`",end:"`"}]},{
    className:"number",variants:[{begin:e.C_NUMBER_RE+"[i]",relevance:1
    },e.C_NUMBER_MODE]},{begin:/:=/},{className:"function",beginKeywords:"func",
    end:"\\s*(\\{|$)",excludeEnd:!0,contains:[e.TITLE_MODE,{className:"params",
    begin:/\(/,end:/\)/,keywords:n,illegal:/["']/}]}]}}})());hljs.registerLanguage("r",(()=>{"use strict";function e(...e){return"("+((e=>{
    const n=e[e.length-1]
    ;return"object"==typeof n&&n.constructor===Object?(e.splice(e.length-1,1),n):{}
    })(e).capture?"":"?:")+e.map((e=>{return(n=e)?"string"==typeof n?n:n.source:null
    ;var n})).join("|")+")"}return n=>{
    const a=/(?:(?:[a-zA-Z]|\.[._a-zA-Z])[._a-zA-Z0-9]*)|\.(?!\d)/,i=e(/0[xX][0-9a-fA-F]+\.[0-9a-fA-F]*[pP][+-]?\d+i?/,/0[xX][0-9a-fA-F]+(?:[pP][+-]?\d+)?[Li]?/,/(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?[Li]?/),s=/[=!<>:]=|\|\||&&|:::?|<-|<<-|->>|->|\|>|[-+*\/?!$&|:<=>@^~]|\*\*/,t=e(/[()]/,/[{}]/,/\[\[/,/[[\]]/,/\\/,/,/)
    ;return{name:"R",keywords:{$pattern:a,
    keyword:"function if in break next repeat else for while",
    literal:"NULL NA TRUE FALSE Inf NaN NA_integer_|10 NA_real_|10 NA_character_|10 NA_complex_|10",
    built_in:"LETTERS letters month.abb month.name pi T F abs acos acosh all any anyNA Arg as.call as.character as.complex as.double as.environment as.integer as.logical as.null.default as.numeric as.raw asin asinh atan atanh attr attributes baseenv browser c call ceiling class Conj cos cosh cospi cummax cummin cumprod cumsum digamma dim dimnames emptyenv exp expression floor forceAndCall gamma gc.time globalenv Im interactive invisible is.array is.atomic is.call is.character is.complex is.double is.environment is.expression is.finite is.function is.infinite is.integer is.language is.list is.logical is.matrix is.na is.name is.nan is.null is.numeric is.object is.pairlist is.raw is.recursive is.single is.symbol lazyLoadDBfetch length lgamma list log max min missing Mod names nargs nzchar oldClass on.exit pos.to.env proc.time prod quote range Re rep retracemem return round seq_along seq_len seq.int sign signif sin sinh sinpi sqrt standardGeneric substitute sum switch tan tanh tanpi tracemem trigamma trunc unclass untracemem UseMethod xtfrm"
    },contains:[n.COMMENT(/#'/,/$/,{contains:[{scope:"doctag",begin:"@examples",
    starts:{contains:[{begin:/\n/},{begin:/#'\s*(?=@[a-zA-Z]+)/,endsParent:!0},{
    begin:/#'/,end:/$/,excludeBegin:!0}]}},{scope:"doctag",begin:"@param",end:/$/,
    contains:[{scope:"variable",variants:[{begin:a},{begin:/`(?:\\.|[^`\\])+`/}],
    endsParent:!0}]},{scope:"doctag",begin:/@[a-zA-Z]+/},{scope:"keyword",
    begin:/\\[a-zA-Z]+/}]}),n.HASH_COMMENT_MODE,{scope:"string",
    contains:[n.BACKSLASH_ESCAPE],variants:[n.END_SAME_AS_BEGIN({
    begin:/[rR]"(-*)\(/,end:/\)(-*)"/}),n.END_SAME_AS_BEGIN({begin:/[rR]"(-*)\{/,
    end:/\}(-*)"/}),n.END_SAME_AS_BEGIN({begin:/[rR]"(-*)\[/,end:/\](-*)"/
    }),n.END_SAME_AS_BEGIN({begin:/[rR]'(-*)\(/,end:/\)(-*)'/
    }),n.END_SAME_AS_BEGIN({begin:/[rR]'(-*)\{/,end:/\}(-*)'/
    }),n.END_SAME_AS_BEGIN({begin:/[rR]'(-*)\[/,end:/\](-*)'/}),{begin:'"',end:'"',
    relevance:0},{begin:"'",end:"'",relevance:0}]},{relevance:0,variants:[{scope:{
    1:"operator",2:"number"},match:[s,i]},{scope:{1:"operator",2:"number"},
    match:[/%[^%]*%/,i]},{scope:{1:"punctuation",2:"number"},match:[t,i]},{scope:{
    2:"number"},match:[/[^a-zA-Z0-9._]|^/,i]}]},{scope:{3:"operator"},
    match:[a,/\s+/,/<-/,/\s+/]},{scope:"operator",relevance:0,variants:[{match:s},{
    match:/%[^%]*%/}]},{scope:"punctuation",relevance:0,match:t},{begin:"`",end:"`",
    contains:[{begin:/\\./}]}]}}})());hljs.registerLanguage("json",(()=>{"use strict";return e=>({name:"JSON",
    contains:[{className:"attr",begin:/"(\\.|[^\\"\r\n])*"(?=\s*:)/,relevance:1.01
    },{match:/[{}[\],:]/,className:"punctuation",relevance:0},e.QUOTE_STRING_MODE,{
    beginKeywords:"true false null"
    },e.C_NUMBER_MODE,e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE],illegal:"\\S"})
    })());hljs.registerLanguage("erlang-repl",(()=>{"use strict";function e(...e){
    return e.map((e=>{return(n=e)?"string"==typeof n?n:n.source:null;var n
    })).join("")}return n=>({name:"Erlang REPL",keywords:{
    built_in:"spawn spawn_link self",
    keyword:"after and andalso|10 band begin bnot bor bsl bsr bxor case catch cond div end fun if let not of or orelse|10 query receive rem try when xor"
    },contains:[{className:"meta",begin:"^[0-9]+> ",relevance:10
    },n.COMMENT("%","$"),{className:"number",
    begin:"\\b(\\d+(_\\d+)*#[a-fA-F0-9]+(_[a-fA-F0-9]+)*|\\d+(_\\d+)*(\\.\\d+(_\\d+)*)?([eE][-+]?\\d+)?)",
    relevance:0},n.APOS_STRING_MODE,n.QUOTE_STRING_MODE,{
    begin:e(/\?(::)?/,/([A-Z]\w*)/,/((::)[A-Z]\w*)*/)},{begin:"->"},{begin:"ok"},{
    begin:"!"},{
    begin:"(\\b[a-z'][a-zA-Z0-9_']*:[a-z'][a-zA-Z0-9_']*)|(\\b[a-z'][a-zA-Z0-9_']*)",
    relevance:0},{begin:"[A-Z][a-zA-Z0-9_']*",relevance:0}]})})());hljs.registerLanguage("haskell",(()=>{"use strict";return e=>{const n={
    variants:[e.COMMENT("--","$"),e.COMMENT(/\{-/,/-\}/,{contains:["self"]})]},a={
    className:"meta",begin:/\{-#/,end:/#-\}/},i={className:"meta",begin:"^#",end:"$"
    },s={className:"type",begin:"\\b[A-Z][\\w']*",relevance:0},l={begin:"\\(",
    end:"\\)",illegal:'"',contains:[a,i,{className:"type",
    begin:"\\b[A-Z][\\w]*(\\((\\.\\.|,|\\w+)\\))?"},e.inherit(e.TITLE_MODE,{
    begin:"[_a-z][\\w']*"}),n]},t="([0-9a-fA-F]_*)+",c={className:"number",
    relevance:0,variants:[{
    match:"\\b(([0-9]_*)+)(\\.(([0-9]_*)+))?([eE][+-]?(([0-9]_*)+))?\\b"},{
    match:`\\b0[xX]_*(${t})(\\.(${t}))?([pP][+-]?(([0-9]_*)+))?\\b`},{
    match:"\\b0[oO](([0-7]_*)+)\\b"},{match:"\\b0[bB](([01]_*)+)\\b"}]};return{
    name:"Haskell",aliases:["hs"],
    keywords:"let in if then else case of where do module import hiding qualified type data newtype deriving class instance as default infix infixl infixr foreign export ccall stdcall cplusplus jvm dotnet safe unsafe family forall mdo proc rec",
    contains:[{beginKeywords:"module",end:"where",keywords:"module where",
    contains:[l,n],illegal:"\\W\\.|;"},{begin:"\\bimport\\b",end:"$",
    keywords:"import qualified as hiding",contains:[l,n],illegal:"\\W\\.|;"},{
    className:"class",begin:"^(\\s*)?(class|instance)\\b",end:"where",
    keywords:"class family instance where",contains:[s,l,n]},{className:"class",
    begin:"\\b(data|(new)?type)\\b",end:"$",
    keywords:"data family type newtype deriving",contains:[a,s,l,{begin:/\{/,
    end:/\}/,contains:l.contains},n]},{beginKeywords:"default",end:"$",
    contains:[s,l,n]},{beginKeywords:"infix infixl infixr",end:"$",
    contains:[e.C_NUMBER_MODE,n]},{begin:"\\bforeign\\b",end:"$",
    keywords:"foreign import export ccall stdcall cplusplus jvm dotnet safe unsafe",
    contains:[s,e.QUOTE_STRING_MODE,n]},{className:"meta",
    begin:"#!\\/usr\\/bin\\/env runhaskell",end:"$"
    },a,i,e.QUOTE_STRING_MODE,c,s,e.inherit(e.TITLE_MODE,{begin:"^[_a-z][\\w']*"
    }),n,{begin:"->|<-"}]}}})());hljs.registerLanguage("bash",(()=>{"use strict";function e(...e){
    return e.map((e=>{return(s=e)?"string"==typeof s?s:s.source:null;var s
    })).join("")}return s=>{const n={},t={begin:/\$\{/,end:/\}/,contains:["self",{
    begin:/:-/,contains:[n]}]};Object.assign(n,{className:"variable",variants:[{
    begin:e(/\$[\w\d#@][\w\d_]*/,"(?![\\w\\d])(?![$])")},t]});const a={
    className:"subst",begin:/\$\(/,end:/\)/,contains:[s.BACKSLASH_ESCAPE]},i={
    begin:/<<-?\s*(?=\w+)/,starts:{contains:[s.END_SAME_AS_BEGIN({begin:/(\w+)/,
    end:/(\w+)/,className:"string"})]}},c={className:"string",begin:/"/,end:/"/,
    contains:[s.BACKSLASH_ESCAPE,n,a]};a.contains.push(c);const o={begin:/\$\(\(/,
    end:/\)\)/,contains:[{begin:/\d+#[0-9a-f]+/,className:"number"},s.NUMBER_MODE,n]
    },r=s.SHEBANG({binary:"(fish|bash|zsh|sh|csh|ksh|tcsh|dash|scsh)",relevance:10
    }),l={className:"function",begin:/\w[\w\d_]*\s*\(\s*\)\s*\{/,returnBegin:!0,
    contains:[s.inherit(s.TITLE_MODE,{begin:/\w[\w\d_]*/})],relevance:0};return{
    name:"Bash",aliases:["sh"],keywords:{$pattern:/\b[a-z._-]+\b/,
    keyword:["if","then","else","elif","fi","for","while","in","do","done","case","esac","function"],
    literal:["true","false"],
    built_in:"break cd continue eval exec exit export getopts hash pwd readonly return shift test times trap umask unset alias bind builtin caller command declare echo enable help let local logout mapfile printf read readarray source type typeset ulimit unalias set shopt autoload bg bindkey bye cap chdir clone comparguments compcall compctl compdescribe compfiles compgroups compquote comptags comptry compvalues dirs disable disown echotc echoti emulate fc fg float functions getcap getln history integer jobs kill limit log noglob popd print pushd pushln rehash sched setcap setopt stat suspend ttyctl unfunction unhash unlimit unsetopt vared wait whence where which zcompile zformat zftp zle zmodload zparseopts zprof zpty zregexparse zsocket zstyle ztcp"
    },contains:[r,s.SHEBANG(),l,o,s.HASH_COMMENT_MODE,i,c,{className:"",begin:/\\"/
    },{className:"string",begin:/'/,end:/'/},n]}}})());hljs.registerLanguage("php",(()=>{"use strict";return e=>{const r={
    className:"variable",
    begin:"\\$+[a-zA-Z_\x7f-\xff][a-zA-Z0-9_\x7f-\xff]*(?![A-Za-z0-9])(?![$])"},t={
    className:"meta",variants:[{begin:/<\?php/,relevance:10},{begin:/<\?[=]?/},{
    begin:/\?>/}]},a={className:"subst",variants:[{begin:/\$\w+/},{begin:/\{\$/,
    end:/\}/}]},n=e.inherit(e.APOS_STRING_MODE,{illegal:null
    }),i=e.inherit(e.QUOTE_STRING_MODE,{illegal:null,
    contains:e.QUOTE_STRING_MODE.contains.concat(a)}),o=e.END_SAME_AS_BEGIN({
    begin:/<<<[ \t]*(\w+)\n/,end:/[ \t]*(\w+)\b/,
    contains:e.QUOTE_STRING_MODE.contains.concat(a)}),l={className:"string",
    contains:[e.BACKSLASH_ESCAPE,t],variants:[e.inherit(n,{begin:"b'",end:"'"
    }),e.inherit(i,{begin:'b"',end:'"'}),i,n,o]},s={className:"number",variants:[{
    begin:"\\b0b[01]+(?:_[01]+)*\\b"},{begin:"\\b0o[0-7]+(?:_[0-7]+)*\\b"},{
    begin:"\\b0x[\\da-f]+(?:_[\\da-f]+)*\\b"},{
    begin:"(?:\\b\\d+(?:_\\d+)*(\\.(?:\\d+(?:_\\d+)*))?|\\B\\.\\d+)(?:e[+-]?\\d+)?"
    }],relevance:0},c={
    keyword:"__CLASS__ __DIR__ __FILE__ __FUNCTION__ __LINE__ __METHOD__ __NAMESPACE__ __TRAIT__ die echo exit include include_once print require require_once array abstract and as binary bool boolean break callable case catch class clone const continue declare default do double else elseif empty enddeclare endfor endforeach endif endswitch endwhile enum eval extends final finally float for foreach from global goto if implements instanceof insteadof int integer interface isset iterable list match|0 mixed new object or private protected public real return string switch throw trait try unset use var void while xor yield",
    literal:"false null true",
    built_in:"Error|0 AppendIterator ArgumentCountError ArithmeticError ArrayIterator ArrayObject AssertionError BadFunctionCallException BadMethodCallException CachingIterator CallbackFilterIterator CompileError Countable DirectoryIterator DivisionByZeroError DomainException EmptyIterator ErrorException Exception FilesystemIterator FilterIterator GlobIterator InfiniteIterator InvalidArgumentException IteratorIterator LengthException LimitIterator LogicException MultipleIterator NoRewindIterator OutOfBoundsException OutOfRangeException OuterIterator OverflowException ParentIterator ParseError RangeException RecursiveArrayIterator RecursiveCachingIterator RecursiveCallbackFilterIterator RecursiveDirectoryIterator RecursiveFilterIterator RecursiveIterator RecursiveIteratorIterator RecursiveRegexIterator RecursiveTreeIterator RegexIterator RuntimeException SeekableIterator SplDoublyLinkedList SplFileInfo SplFileObject SplFixedArray SplHeap SplMaxHeap SplMinHeap SplObjectStorage SplObserver SplObserver SplPriorityQueue SplQueue SplStack SplSubject SplSubject SplTempFileObject TypeError UnderflowException UnexpectedValueException UnhandledMatchError ArrayAccess Closure Generator Iterator IteratorAggregate Serializable Stringable Throwable Traversable WeakReference WeakMap Directory __PHP_Incomplete_Class parent php_user_filter self static stdClass"
    };return{case_insensitive:!0,keywords:c,
    contains:[e.HASH_COMMENT_MODE,e.COMMENT("//","$",{contains:[t]
    }),e.COMMENT("/\\*","\\*/",{contains:[{className:"doctag",begin:"@[A-Za-z]+"}]
    }),e.COMMENT("__halt_compiler.+?;",!1,{endsWithParent:!0,
    keywords:"__halt_compiler"}),t,{className:"keyword",begin:/\$this\b/},r,{
    begin:/(::|->)+[a-zA-Z_\x7f-\xff][a-zA-Z0-9_\x7f-\xff]*/},{className:"function",
    relevance:0,beginKeywords:"fn function",end:/[;{]/,excludeEnd:!0,
    illegal:"[$%\\[]",contains:[{beginKeywords:"use"},e.UNDERSCORE_TITLE_MODE,{
    begin:"=>",endsParent:!0},{className:"params",begin:"\\(",end:"\\)",
    excludeBegin:!0,excludeEnd:!0,keywords:c,
    contains:["self",r,e.C_BLOCK_COMMENT_MODE,l,s]}]},{className:"class",variants:[{
    beginKeywords:"enum",illegal:/[($"]/},{beginKeywords:"class interface trait",
    illegal:/[:($"]/}],relevance:0,end:/\{/,excludeEnd:!0,contains:[{
    beginKeywords:"extends implements"},e.UNDERSCORE_TITLE_MODE]},{
    beginKeywords:"namespace",relevance:0,end:";",illegal:/[.']/,
    contains:[e.UNDERSCORE_TITLE_MODE]},{beginKeywords:"use",relevance:0,end:";",
    contains:[e.UNDERSCORE_TITLE_MODE]},l,s]}}})());hljs.registerLanguage("xml",(()=>{"use strict";function e(e){
    return e?"string"==typeof e?e:e.source:null}function n(e){return a("(?=",e,")")}
    function a(...n){return n.map((n=>e(n))).join("")}function s(...n){
    return"("+((e=>{const n=e[e.length-1]
    ;return"object"==typeof n&&n.constructor===Object?(e.splice(e.length-1,1),n):{}
    })(n).capture?"":"?:")+n.map((n=>e(n))).join("|")+")"}return e=>{
    const t=a(/[A-Z_]/,a("(?:",/[A-Z0-9_.-]*:/,")?"),/[A-Z0-9_.-]*/),i={
    className:"symbol",begin:/&[a-z]+;|&#[0-9]+;|&#x[a-f0-9]+;/},c={begin:/\s/,
    contains:[{className:"keyword",begin:/#?[a-z_][a-z1-9_-]+/,illegal:/\n/}]
    },r=e.inherit(c,{begin:/\(/,end:/\)/}),l=e.inherit(e.APOS_STRING_MODE,{
    className:"string"}),g=e.inherit(e.QUOTE_STRING_MODE,{className:"string"}),m={
    endsWithParent:!0,illegal:/</,relevance:0,contains:[{className:"attr",
    begin:/[A-Za-z0-9._:-]+/,relevance:0},{begin:/=\s*/,relevance:0,contains:[{
    className:"string",endsParent:!0,variants:[{begin:/"/,end:/"/,contains:[i]},{
    begin:/'/,end:/'/,contains:[i]},{begin:/[^\s"'=<>`]+/}]}]}]};return{
    name:"HTML, XML",
    aliases:["html","xhtml","rss","atom","xjb","xsd","xsl","plist","wsf","svg"],
    case_insensitive:!0,contains:[{className:"meta",begin:/<![a-z]/,end:/>/,
    relevance:10,contains:[c,g,l,r,{begin:/\[/,end:/\]/,contains:[{className:"meta",
    begin:/<![a-z]/,end:/>/,contains:[c,r,g,l]}]}]},e.COMMENT(/<!--/,/-->/,{
    relevance:10}),{begin:/<!\[CDATA\[/,end:/\]\]>/,relevance:10},i,{
    className:"meta",begin:/<\?xml/,end:/\?>/,relevance:10},{className:"tag",
    begin:/<style(?=\s|>)/,end:/>/,keywords:{name:"style"},contains:[m],starts:{
    end:/<\/style>/,returnEnd:!0,subLanguage:["css","xml"]}},{className:"tag",
    begin:/<script(?=\s|>)/,end:/>/,keywords:{name:"script"},contains:[m],starts:{
    end:/<\/script>/,returnEnd:!0,subLanguage:["javascript","handlebars","xml"]}},{
    className:"tag",begin:/<>|<\/>/},{className:"tag",
    begin:a(/</,n(a(t,s(/\/>/,/>/,/\s/)))),end:/\/?>/,contains:[{className:"name",
    begin:t,relevance:0,starts:m}]},{className:"tag",begin:a(/<\//,n(a(t,/>/))),
    contains:[{className:"name",begin:t,relevance:0},{begin:/>/,relevance:0,
    endsParent:!0}]}]}}})());hljs.registerLanguage("vbscript",(()=>{"use strict";function e(e){
    return e?"string"==typeof e?e:e.source:null}function t(...t){
    return t.map((t=>e(t))).join("")}function r(...t){return"("+((e=>{
    const t=e[e.length-1]
    ;return"object"==typeof t&&t.constructor===Object?(e.splice(e.length-1,1),t):{}
    })(t).capture?"":"?:")+t.map((t=>e(t))).join("|")+")"}return e=>{
    const n=["lcase","month","vartype","instrrev","ubound","setlocale","getobject","rgb","getref","string","weekdayname","rnd","dateadd","monthname","now","day","minute","isarray","cbool","round","formatcurrency","conversions","csng","timevalue","second","year","space","abs","clng","timeserial","fixs","len","asc","isempty","maths","dateserial","atn","timer","isobject","filter","weekday","datevalue","ccur","isdate","instr","datediff","formatdatetime","replace","isnull","right","sgn","array","snumeric","log","cdbl","hex","chr","lbound","msgbox","ucase","getlocale","cos","cdate","cbyte","rtrim","join","hour","oct","typename","trim","strcomp","int","createobject","loadpicture","tan","formatnumber","mid","split","cint","sin","datepart","ltrim","sqr","time","derived","eval","date","formatpercent","exp","inputbox","left","ascw","chrw","regexp","cstr","err"]
    ;return{name:"VBScript",aliases:["vbs"],case_insensitive:!0,keywords:{
    keyword:["call","class","const","dim","do","loop","erase","execute","executeglobal","exit","for","each","next","function","if","then","else","on","error","option","explicit","new","private","property","let","get","public","randomize","redim","rem","select","case","set","stop","sub","while","wend","with","end","to","elseif","is","or","xor","and","not","class_initialize","class_terminate","default","preserve","in","me","byval","byref","step","resume","goto"],
    built_in:["server","response","request","scriptengine","scriptenginebuildversion","scriptengineminorversion","scriptenginemajorversion"],
    literal:["true","false","null","nothing","empty"]},illegal:"//",contains:[{
    begin:t(r(...n),"\\s*\\("),relevance:0,keywords:{built_in:n}
    },e.inherit(e.QUOTE_STRING_MODE,{contains:[{begin:'""'}]}),e.COMMENT(/'/,/$/,{
    relevance:0}),e.C_NUMBER_MODE]}}})());hljs.registerLanguage("vbscript-html",(()=>{"use strict";return e=>({
    name:"VBScript in HTML",subLanguage:"xml",contains:[{begin:"<%",end:"%>",
    subLanguage:"vbscript"}]})})());hljs.registerLanguage("perl",(()=>{"use strict";function e(e){
    return e?"string"==typeof e?e:e.source:null}function n(...n){
    return n.map((n=>e(n))).join("")}function t(...n){return"("+((e=>{
    const n=e[e.length-1]
    ;return"object"==typeof n&&n.constructor===Object?(e.splice(e.length-1,1),n):{}
    })(n).capture?"":"?:")+n.map((n=>e(n))).join("|")+")"}return e=>{
    const r=/[dualxmsipngr]{0,12}/,s={$pattern:/[\w.]+/,
    keyword:"abs accept alarm and atan2 bind binmode bless break caller chdir chmod chomp chop chown chr chroot close closedir connect continue cos crypt dbmclose dbmopen defined delete die do dump each else elsif endgrent endhostent endnetent endprotoent endpwent endservent eof eval exec exists exit exp fcntl fileno flock for foreach fork format formline getc getgrent getgrgid getgrnam gethostbyaddr gethostbyname gethostent getlogin getnetbyaddr getnetbyname getnetent getpeername getpgrp getpriority getprotobyname getprotobynumber getprotoent getpwent getpwnam getpwuid getservbyname getservbyport getservent getsockname getsockopt given glob gmtime goto grep gt hex if index int ioctl join keys kill last lc lcfirst length link listen local localtime log lstat lt ma map mkdir msgctl msgget msgrcv msgsnd my ne next no not oct open opendir or ord our pack package pipe pop pos print printf prototype push q|0 qq quotemeta qw qx rand read readdir readline readlink readpipe recv redo ref rename require reset return reverse rewinddir rindex rmdir say scalar seek seekdir select semctl semget semop send setgrent sethostent setnetent setpgrp setpriority setprotoent setpwent setservent setsockopt shift shmctl shmget shmread shmwrite shutdown sin sleep socket socketpair sort splice split sprintf sqrt srand stat state study sub substr symlink syscall sysopen sysread sysseek system syswrite tell telldir tie tied time times tr truncate uc ucfirst umask undef unless unlink unpack unshift untie until use utime values vec wait waitpid wantarray warn when while write x|0 xor y|0"
    },i={className:"subst",begin:"[$@]\\{",end:"\\}",keywords:s},a={begin:/->\{/,
    end:/\}/},o={variants:[{begin:/\$\d/},{
    begin:n(/[$%@](\^\w\b|#\w+(::\w+)*|\{\w+\}|\w+(::\w*)*)/,"(?![A-Za-z])(?![@$%])")
    },{begin:/[$%@][^\s\w{]/,relevance:0}]
    },c=[e.BACKSLASH_ESCAPE,i,o],g=[/!/,/\//,/\|/,/\?/,/'/,/"/,/#/],l=(e,t,s="\\1")=>{
    const i="\\1"===s?s:n(s,t)
    ;return n(n("(?:",e,")"),t,/(?:\\.|[^\\\/])*?/,i,/(?:\\.|[^\\\/])*?/,s,r)
    },d=(e,t,s)=>n(n("(?:",e,")"),t,/(?:\\.|[^\\\/])*?/,s,r),p=[o,e.HASH_COMMENT_MODE,e.COMMENT(/^=\w/,/=cut/,{
    endsWithParent:!0}),a,{className:"string",contains:c,variants:[{
    begin:"q[qwxr]?\\s*\\(",end:"\\)",relevance:5},{begin:"q[qwxr]?\\s*\\[",
    end:"\\]",relevance:5},{begin:"q[qwxr]?\\s*\\{",end:"\\}",relevance:5},{
    begin:"q[qwxr]?\\s*\\|",end:"\\|",relevance:5},{begin:"q[qwxr]?\\s*<",end:">",
    relevance:5},{begin:"qw\\s+q",end:"q",relevance:5},{begin:"'",end:"'",
    contains:[e.BACKSLASH_ESCAPE]},{begin:'"',end:'"'},{begin:"`",end:"`",
    contains:[e.BACKSLASH_ESCAPE]},{begin:/\{\w+\}/,relevance:0},{
    begin:"-?\\w+\\s*=>",relevance:0}]},{className:"number",
    begin:"(\\b0[0-7_]+)|(\\b0x[0-9a-fA-F_]+)|(\\b[1-9][0-9_]*(\\.[0-9_]+)?)|[0_]\\b",
    relevance:0},{
    begin:"(\\/\\/|"+e.RE_STARTERS_RE+"|\\b(split|return|print|reverse|grep)\\b)\\s*",
    keywords:"split return print reverse grep",relevance:0,
    contains:[e.HASH_COMMENT_MODE,{className:"regexp",variants:[{
    begin:l("s|tr|y",t(...g,{capture:!0}))},{begin:l("s|tr|y","\\(","\\)")},{
    begin:l("s|tr|y","\\[","\\]")},{begin:l("s|tr|y","\\{","\\}")}],relevance:2},{
    className:"regexp",variants:[{begin:/(m|qr)\/\//,relevance:0},{
    begin:d("(?:m|qr)?",/\//,/\//)},{begin:d("m|qr",t(...g,{capture:!0}),/\1/)},{
    begin:d("m|qr",/\(/,/\)/)},{begin:d("m|qr",/\[/,/\]/)},{
    begin:d("m|qr",/\{/,/\}/)}]}]},{className:"function",beginKeywords:"sub",
    end:"(\\s*\\(.*?\\))?[;{]",excludeEnd:!0,relevance:5,contains:[e.TITLE_MODE]},{
    begin:"-\\w\\b",relevance:0},{begin:"^__DATA__$",end:"^__END__$",
    subLanguage:"mojolicious",contains:[{begin:"^@@.*",end:"$",className:"comment"}]
    }];return i.contains=p,a.contains=p,{name:"Perl",aliases:["pl","pm"],keywords:s,
    contains:p}}})());hljs.registerLanguage("csharp",(()=>{"use strict";return e=>{const n={
    keyword:["abstract","as","base","break","case","class","const","continue","do","else","event","explicit","extern","finally","fixed","for","foreach","goto","if","implicit","in","interface","internal","is","lock","namespace","new","operator","out","override","params","private","protected","public","readonly","record","ref","return","sealed","sizeof","stackalloc","static","struct","switch","this","throw","try","typeof","unchecked","unsafe","using","virtual","void","volatile","while"].concat(["add","alias","and","ascending","async","await","by","descending","equals","from","get","global","group","init","into","join","let","nameof","not","notnull","on","or","orderby","partial","remove","select","set","unmanaged","value|0","var","when","where","with","yield"]),
    built_in:["bool","byte","char","decimal","delegate","double","dynamic","enum","float","int","long","nint","nuint","object","sbyte","short","string","ulong","uint","ushort"],
    literal:["default","false","null","true"]},i=e.inherit(e.TITLE_MODE,{
    begin:"[a-zA-Z](\\.?\\w)*"}),a={className:"number",variants:[{
    begin:"\\b(0b[01']+)"},{
    begin:"(-?)\\b([\\d']+(\\.[\\d']*)?|\\.[\\d']+)(u|U|l|L|ul|UL|f|F|b|B)"},{
    begin:"(-?)(\\b0[xX][a-fA-F0-9']+|(\\b[\\d']+(\\.[\\d']*)?|\\.[\\d']+)([eE][-+]?[\\d']+)?)"
    }],relevance:0},s={className:"string",begin:'@"',end:'"',contains:[{begin:'""'}]
    },t=e.inherit(s,{illegal:/\n/}),r={className:"subst",begin:/\{/,end:/\}/,
    keywords:n},l=e.inherit(r,{illegal:/\n/}),c={className:"string",begin:/\$"/,
    end:'"',illegal:/\n/,contains:[{begin:/\{\{/},{begin:/\}\}/
    },e.BACKSLASH_ESCAPE,l]},o={className:"string",begin:/\$@"/,end:'"',contains:[{
    begin:/\{\{/},{begin:/\}\}/},{begin:'""'},r]},d=e.inherit(o,{illegal:/\n/,
    contains:[{begin:/\{\{/},{begin:/\}\}/},{begin:'""'},l]})
    ;r.contains=[o,c,s,e.APOS_STRING_MODE,e.QUOTE_STRING_MODE,a,e.C_BLOCK_COMMENT_MODE],
    l.contains=[d,c,t,e.APOS_STRING_MODE,e.QUOTE_STRING_MODE,a,e.inherit(e.C_BLOCK_COMMENT_MODE,{
    illegal:/\n/})];const g={variants:[o,c,s,e.APOS_STRING_MODE,e.QUOTE_STRING_MODE]
    },E={begin:"<",end:">",contains:[{beginKeywords:"in out"},i]
    },_=e.IDENT_RE+"(<"+e.IDENT_RE+"(\\s*,\\s*"+e.IDENT_RE+")*>)?(\\[\\])?",b={
    begin:"@"+e.IDENT_RE,relevance:0};return{name:"C#",aliases:["cs","c#"],
    keywords:n,illegal:/::/,contains:[e.COMMENT("///","$",{returnBegin:!0,
    contains:[{className:"doctag",variants:[{begin:"///",relevance:0},{
    begin:"\x3c!--|--\x3e"},{begin:"</?",end:">"}]}]
    }),e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE,{className:"meta",begin:"#",
    end:"$",keywords:{
    keyword:"if else elif endif define undef warning error line region endregion pragma checksum"
    }},g,a,{beginKeywords:"class interface",relevance:0,end:/[{;=]/,
    illegal:/[^\s:,]/,contains:[{beginKeywords:"where class"
    },i,E,e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE]},{beginKeywords:"namespace",
    relevance:0,end:/[{;=]/,illegal:/[^\s:]/,
    contains:[i,e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE]},{
    beginKeywords:"record",relevance:0,end:/[{;=]/,illegal:/[^\s:]/,
    contains:[i,E,e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE]},{className:"meta",
    begin:"^\\s*\\[(?=[\\w])",excludeBegin:!0,end:"\\]",excludeEnd:!0,contains:[{
    className:"string",begin:/"/,end:/"/}]},{
    beginKeywords:"new return throw await else",relevance:0},{className:"function",
    begin:"("+_+"\\s+)+"+e.IDENT_RE+"\\s*(<.+>\\s*)?\\(",returnBegin:!0,
    end:/\s*[{;=]/,excludeEnd:!0,keywords:n,contains:[{
    beginKeywords:"public private protected static internal protected abstract async extern override unsafe virtual new sealed partial",
    relevance:0},{begin:e.IDENT_RE+"\\s*(<.+>\\s*)?\\(",returnBegin:!0,
    contains:[e.TITLE_MODE,E],relevance:0},{className:"params",begin:/\(/,end:/\)/,
    excludeBegin:!0,excludeEnd:!0,keywords:n,relevance:0,
    contains:[g,a,e.C_BLOCK_COMMENT_MODE]
    },e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE]},b]}}})());hljs.registerLanguage("groovy",(()=>{"use strict";function e(e,n={}){
    return n.variants=e,n}return n=>{
    const a="[A-Za-z0-9_$]+",t=e([n.C_LINE_COMMENT_MODE,n.C_BLOCK_COMMENT_MODE,n.COMMENT("/\\*\\*","\\*/",{
    relevance:0,contains:[{begin:/\w+@/,relevance:0},{className:"doctag",
    begin:"@[A-Za-z]+"}]})]),s={className:"regexp",begin:/~?\/[^\/\n]+\//,
    contains:[n.BACKSLASH_ESCAPE]
    },i=e([n.BINARY_NUMBER_MODE,n.C_NUMBER_MODE]),r=e([{begin:/"""/,end:/"""/},{
    begin:/'''/,end:/'''/},{begin:"\\$/",end:"/\\$",relevance:10
    },n.APOS_STRING_MODE,n.QUOTE_STRING_MODE],{className:"string"});return{
    name:"Groovy",keywords:{built_in:"this super",literal:"true false null",
    keyword:"byte short char int long boolean float double void def as in assert trait abstract static volatile transient public private protected synchronized final class interface enum if else for while switch case break default continue throw throws try catch finally implements extends new import package return instanceof"
    },contains:[n.SHEBANG({binary:"groovy",relevance:10}),t,r,s,i,{
    className:"class",beginKeywords:"class interface trait enum",end:/\{/,
    illegal:":",contains:[{beginKeywords:"extends implements"
    },n.UNDERSCORE_TITLE_MODE]},{className:"meta",begin:"@[A-Za-z]+",relevance:0},{
    className:"attr",begin:a+"[ \t]*:",relevance:0},{begin:/\?/,end:/:/,relevance:0,
    contains:[t,r,s,i,"self"]},{className:"symbol",
    begin:"^[ \t]*"+(l=a+":",((...e)=>e.map((e=>(e=>e?"string"==typeof e?e:e.source:null)(e))).join(""))("(?=",l,")")),
    excludeBegin:!0,end:a+":",relevance:0}],illegal:/#|<\//};var l}})());hljs.registerLanguage("css",(()=>{"use strict"
    ;const e=["a","abbr","address","article","aside","audio","b","blockquote","body","button","canvas","caption","cite","code","dd","del","details","dfn","div","dl","dt","em","fieldset","figcaption","figure","footer","form","h1","h2","h3","h4","h5","h6","header","hgroup","html","i","iframe","img","input","ins","kbd","label","legend","li","main","mark","menu","nav","object","ol","p","q","quote","samp","section","span","strong","summary","sup","table","tbody","td","textarea","tfoot","th","thead","time","tr","ul","var","video"],t=["any-hover","any-pointer","aspect-ratio","color","color-gamut","color-index","device-aspect-ratio","device-height","device-width","display-mode","forced-colors","grid","height","hover","inverted-colors","monochrome","orientation","overflow-block","overflow-inline","pointer","prefers-color-scheme","prefers-contrast","prefers-reduced-motion","prefers-reduced-transparency","resolution","scan","scripting","update","width","min-width","max-width","min-height","max-height"],i=["active","any-link","blank","checked","current","default","defined","dir","disabled","drop","empty","enabled","first","first-child","first-of-type","fullscreen","future","focus","focus-visible","focus-within","has","host","host-context","hover","indeterminate","in-range","invalid","is","lang","last-child","last-of-type","left","link","local-link","not","nth-child","nth-col","nth-last-child","nth-last-col","nth-last-of-type","nth-of-type","only-child","only-of-type","optional","out-of-range","past","placeholder-shown","read-only","read-write","required","right","root","scope","target","target-within","user-invalid","valid","visited","where"],o=["after","backdrop","before","cue","cue-region","first-letter","first-line","grammar-error","marker","part","placeholder","selection","slotted","spelling-error"],r=["align-content","align-items","align-self","animation","animation-delay","animation-direction","animation-duration","animation-fill-mode","animation-iteration-count","animation-name","animation-play-state","animation-timing-function","auto","backface-visibility","background","background-attachment","background-clip","background-color","background-image","background-origin","background-position","background-repeat","background-size","border","border-bottom","border-bottom-color","border-bottom-left-radius","border-bottom-right-radius","border-bottom-style","border-bottom-width","border-collapse","border-color","border-image","border-image-outset","border-image-repeat","border-image-slice","border-image-source","border-image-width","border-left","border-left-color","border-left-style","border-left-width","border-radius","border-right","border-right-color","border-right-style","border-right-width","border-spacing","border-style","border-top","border-top-color","border-top-left-radius","border-top-right-radius","border-top-style","border-top-width","border-width","bottom","box-decoration-break","box-shadow","box-sizing","break-after","break-before","break-inside","caption-side","clear","clip","clip-path","color","column-count","column-fill","column-gap","column-rule","column-rule-color","column-rule-style","column-rule-width","column-span","column-width","columns","content","counter-increment","counter-reset","cursor","direction","display","empty-cells","filter","flex","flex-basis","flex-direction","flex-flow","flex-grow","flex-shrink","flex-wrap","float","font","font-display","font-family","font-feature-settings","font-kerning","font-language-override","font-size","font-size-adjust","font-smoothing","font-stretch","font-style","font-variant","font-variant-ligatures","font-variation-settings","font-weight","height","hyphens","icon","image-orientation","image-rendering","image-resolution","ime-mode","inherit","initial","justify-content","left","letter-spacing","line-height","list-style","list-style-image","list-style-position","list-style-type","margin","margin-bottom","margin-left","margin-right","margin-top","marks","mask","max-height","max-width","min-height","min-width","nav-down","nav-index","nav-left","nav-right","nav-up","none","normal","object-fit","object-position","opacity","order","orphans","outline","outline-color","outline-offset","outline-style","outline-width","overflow","overflow-wrap","overflow-x","overflow-y","padding","padding-bottom","padding-left","padding-right","padding-top","page-break-after","page-break-before","page-break-inside","perspective","perspective-origin","pointer-events","position","quotes","resize","right","src","tab-size","table-layout","text-align","text-align-last","text-decoration","text-decoration-color","text-decoration-line","text-decoration-style","text-indent","text-overflow","text-rendering","text-shadow","text-transform","text-underline-position","top","transform","transform-origin","transform-style","transition","transition-delay","transition-duration","transition-property","transition-timing-function","unicode-bidi","vertical-align","visibility","white-space","widows","width","word-break","word-spacing","word-wrap","z-index"].reverse()
    ;return n=>{const a=(e=>({IMPORTANT:{scope:"meta",begin:"!important"},HEXCOLOR:{
    scope:"number",begin:"#([a-fA-F0-9]{6}|[a-fA-F0-9]{3})"},
    ATTRIBUTE_SELECTOR_MODE:{scope:"selector-attr",begin:/\[/,end:/\]/,illegal:"$",
    contains:[e.APOS_STRING_MODE,e.QUOTE_STRING_MODE]},CSS_NUMBER_MODE:{
    scope:"number",
    begin:e.NUMBER_RE+"(%|em|ex|ch|rem|vw|vh|vmin|vmax|cm|mm|in|pt|pc|px|deg|grad|rad|turn|s|ms|Hz|kHz|dpi|dpcm|dppx)?",
    relevance:0}}))(n),l=[n.APOS_STRING_MODE,n.QUOTE_STRING_MODE];return{name:"CSS",
    case_insensitive:!0,illegal:/[=|'\$]/,keywords:{keyframePosition:"from to"},
    classNameAliases:{keyframePosition:"selector-tag"},
    contains:[n.C_BLOCK_COMMENT_MODE,{begin:/-(webkit|moz|ms|o)-(?=[a-z])/
    },a.CSS_NUMBER_MODE,{className:"selector-id",begin:/#[A-Za-z0-9_-]+/,relevance:0
    },{className:"selector-class",begin:"\\.[a-zA-Z-][a-zA-Z0-9_-]*",relevance:0
    },a.ATTRIBUTE_SELECTOR_MODE,{className:"selector-pseudo",variants:[{
    begin:":("+i.join("|")+")"},{begin:"::("+o.join("|")+")"}]},{
    className:"attribute",begin:"\\b("+r.join("|")+")\\b"},{begin:":",end:"[;}]",
    contains:[a.HEXCOLOR,a.IMPORTANT,a.CSS_NUMBER_MODE,...l,{
    begin:/(url|data-uri)\(/,end:/\)/,relevance:0,keywords:{built_in:"url data-uri"
    },contains:[{className:"string",begin:/[^)]/,endsWithParent:!0,excludeEnd:!0}]
    },{className:"built_in",begin:/[\w-]+(?=\()/}]},{
    begin:(s=/@/,((...e)=>e.map((e=>(e=>e?"string"==typeof e?e:e.source:null)(e))).join(""))("(?=",s,")")),
    end:"[{;]",relevance:0,illegal:/:/,contains:[{className:"keyword",
    begin:/@-?\w[\w]*(-\w+)*/},{begin:/\s/,endsWithParent:!0,excludeEnd:!0,
    relevance:0,keywords:{$pattern:/[a-z-]+/,keyword:"and or not only",
    attribute:t.join(" ")},contains:[{begin:/[a-z-]+(?=:)/,className:"attribute"
    },...l,a.CSS_NUMBER_MODE]}]},{className:"selector-tag",
    begin:"\\b("+e.join("|")+")\\b"}]};var s}})());hljs.registerLanguage("c",(()=>{"use strict";function e(e){
    return((...e)=>e.map((e=>(e=>e?"string"==typeof e?e:e.source:null)(e))).join(""))("(?:",e,")?")
    }return n=>{const t=n.COMMENT("//","$",{contains:[{begin:/\\\n/}]
    }),s="[a-zA-Z_]\\w*::",r="(decltype\\(auto\\)|"+e(s)+"[a-zA-Z_]\\w*"+e("<[^<>]+>")+")",a={
    className:"type",variants:[{begin:"\\b[a-z\\d_]*_t\\b"},{
    match:/\batomic_[a-z]{3,6}\b/}]},i={className:"string",variants:[{
    begin:'(u8?|U|L)?"',end:'"',illegal:"\\n",contains:[n.BACKSLASH_ESCAPE]},{
    begin:"(u8?|U|L)?'(\\\\(x[0-9A-Fa-f]{2}|u[0-9A-Fa-f]{4,8}|[0-7]{3}|\\S)|.)",
    end:"'",illegal:"."},n.END_SAME_AS_BEGIN({
    begin:/(?:u8?|U|L)?R"([^()\\ ]{0,16})\(/,end:/\)([^()\\ ]{0,16})"/})]},l={
    className:"number",variants:[{begin:"\\b(0b[01']+)"},{
    begin:"(-?)\\b([\\d']+(\\.[\\d']*)?|\\.[\\d']+)((ll|LL|l|L)(u|U)?|(u|U)(ll|LL|l|L)?|f|F|b|B)"
    },{
    begin:"(-?)(\\b0[xX][a-fA-F0-9']+|(\\b[\\d']+(\\.[\\d']*)?|\\.[\\d']+)([eE][-+]?[\\d']+)?)"
    }],relevance:0},c={className:"meta",begin:/#\s*[a-z]+\b/,end:/$/,keywords:{
    keyword:"if else elif endif define undef warning error line pragma _Pragma ifdef ifndef include"
    },contains:[{begin:/\\\n/,relevance:0},n.inherit(i,{className:"string"}),{
    className:"string",begin:/<.*?>/},t,n.C_BLOCK_COMMENT_MODE]},o={
    className:"title",begin:e(s)+n.IDENT_RE,relevance:0
    },d=e(s)+n.IDENT_RE+"\\s*\\(",u={
    keyword:["asm","auto","break","case","const","continue","default","do","else","enum","extern","for","fortran","goto","if","inline","register","restrict","return","sizeof","static","struct","switch","typedef","union","volatile","while","_Alignas","_Alignof","_Atomic","_Generic","_Noreturn","_Static_assert","_Thread_local","alignas","alignof","noreturn","static_assert","thread_local","_Pragma"],
    type:["float","double","signed","unsigned","int","short","long","char","void","_Bool","_Complex","_Imaginary","_Decimal32","_Decimal64","_Decimal128","complex","bool","imaginary"],
    literal:"true false NULL",
    built_in:"std string wstring cin cout cerr clog stdin stdout stderr stringstream istringstream ostringstream auto_ptr deque list queue stack vector map set pair bitset multiset multimap unordered_set unordered_map unordered_multiset unordered_multimap priority_queue make_pair array shared_ptr abort terminate abs acos asin atan2 atan calloc ceil cosh cos exit exp fabs floor fmod fprintf fputs free frexp fscanf future isalnum isalpha iscntrl isdigit isgraph islower isprint ispunct isspace isupper isxdigit tolower toupper labs ldexp log10 log malloc realloc memchr memcmp memcpy memset modf pow printf putchar puts scanf sinh sin snprintf sprintf sqrt sscanf strcat strchr strcmp strcpy strcspn strlen strncat strncmp strncpy strpbrk strrchr strspn strstr tanh tan vfprintf vprintf vsprintf endl initializer_list unique_ptr"
    },g=[c,a,t,n.C_BLOCK_COMMENT_MODE,l,i],m={variants:[{begin:/=/,end:/;/},{
    begin:/\(/,end:/\)/},{beginKeywords:"new throw return else",end:/;/}],
    keywords:u,contains:g.concat([{begin:/\(/,end:/\)/,keywords:u,
    contains:g.concat(["self"]),relevance:0}]),relevance:0},_={
    begin:"("+r+"[\\*&\\s]+)+"+d,returnBegin:!0,end:/[{;=]/,excludeEnd:!0,
    keywords:u,illegal:/[^\w\s\*&:<>.]/,contains:[{begin:"decltype\\(auto\\)",
    keywords:u,relevance:0},{begin:d,returnBegin:!0,contains:[n.inherit(o,{
    className:"title.function"})],relevance:0},{relevance:0,match:/,/},{
    className:"params",begin:/\(/,end:/\)/,keywords:u,relevance:0,
    contains:[t,n.C_BLOCK_COMMENT_MODE,i,l,a,{begin:/\(/,end:/\)/,keywords:u,
    relevance:0,contains:["self",t,n.C_BLOCK_COMMENT_MODE,i,l,a]}]
    },a,t,n.C_BLOCK_COMMENT_MODE,c]};return{name:"C",aliases:["h"],keywords:u,
    disableAutodetect:!0,illegal:"</",contains:[].concat(m,_,g,[c,{
    begin:n.IDENT_RE+"::",keywords:u},{className:"class",
    beginKeywords:"enum class struct union",end:/[{;:<>=]/,contains:[{
    beginKeywords:"final class struct"},n.TITLE_MODE]}]),exports:{preprocessor:c,
    strings:i,keywords:u}}}})());hljs.registerLanguage("cpp",(()=>{"use strict";function e(e){
    return t("(?:",e,")?")}function t(...e){return e.map((e=>{
    return(t=e)?"string"==typeof t?t:t.source:null;var t})).join("")}return n=>{
    const a=n.COMMENT("//","$",{contains:[{begin:/\\\n/}]
    }),r="[a-zA-Z_]\\w*::",i="(?!struct)(decltype\\(auto\\)|"+e(r)+"[a-zA-Z_]\\w*"+e("<[^<>]+>")+")",s={
    className:"type",begin:"\\b[a-z\\d_]*_t\\b"},c={className:"string",variants:[{
    begin:'(u8?|U|L)?"',end:'"',illegal:"\\n",contains:[n.BACKSLASH_ESCAPE]},{
    begin:"(u8?|U|L)?'(\\\\(x[0-9A-Fa-f]{2}|u[0-9A-Fa-f]{4,8}|[0-7]{3}|\\S)|.)",
    end:"'",illegal:"."},n.END_SAME_AS_BEGIN({
    begin:/(?:u8?|U|L)?R"([^()\\ ]{0,16})\(/,end:/\)([^()\\ ]{0,16})"/})]},o={
    className:"number",variants:[{begin:"\\b(0b[01']+)"},{
    begin:"(-?)\\b([\\d']+(\\.[\\d']*)?|\\.[\\d']+)((ll|LL|l|L)(u|U)?|(u|U)(ll|LL|l|L)?|f|F|b|B)"
    },{
    begin:"(-?)(\\b0[xX][a-fA-F0-9']+|(\\b[\\d']+(\\.[\\d']*)?|\\.[\\d']+)([eE][-+]?[\\d']+)?)"
    }],relevance:0},l={className:"meta",begin:/#\s*[a-z]+\b/,end:/$/,keywords:{
    keyword:"if else elif endif define undef warning error line pragma _Pragma ifdef ifndef include"
    },contains:[{begin:/\\\n/,relevance:0},n.inherit(c,{className:"string"}),{
    className:"string",begin:/<.*?>/},a,n.C_BLOCK_COMMENT_MODE]},u={
    className:"title",begin:e(r)+n.IDENT_RE,relevance:0
    },d=e(r)+n.IDENT_RE+"\\s*\\(",p={
    type:["bool","char","char16_t","char32_t","char8_t","double","float","int","long","short","void","wchar_t"],
    keyword:["alignas","alignof","and","and_eq","asm","atomic_cancel","atomic_commit","atomic_noexcept","auto","bitand","bitor","break","case","catch","class","co_await","co_return","co_yield","compl","concept","const","const_cast|10","consteval","constexpr","constinit","continue","decltype","default","delete","do","dynamic_cast|10","else","enum","explicit","export","extern","false","final","for","friend","goto","if","import","inline","module","mutable","namespace","new","noexcept","not","not_eq","nullptr","operator","or","or_eq","override","private","protected","public","reflexpr","register","reinterpret_cast|10","requires","return","signed","sizeof","static","static_assert","static_cast|10","struct","switch","synchronized","template","this","thread_local","throw","transaction_safe","transaction_safe_dynamic","true","try","typedef","typeid","typename","union","unsigned","using","virtual","volatile","while","xor","xor_eq,"],
    literal:["NULL","false","nullopt","nullptr","true"],built_in:["_Pragma"],
    _type_hints:["any","auto_ptr","barrier","binary_semaphore","bitset","complex","condition_variable","condition_variable_any","counting_semaphore","deque","false_type","future","imaginary","initializer_list","istringstream","jthread","latch","lock_guard","multimap","multiset","mutex","optional","ostringstream","packaged_task","pair","promise","priority_queue","queue","recursive_mutex","recursive_timed_mutex","scoped_lock","set","shared_future","shared_lock","shared_mutex","shared_timed_mutex","shared_ptr","stack","string_view","stringstream","timed_mutex","thread","true_type","tuple","unique_lock","unique_ptr","unordered_map","unordered_multimap","unordered_multiset","unordered_set","variant","vector","weak_ptr","wstring","wstring_view"]
    },_={className:"function.dispatch",relevance:0,keywords:{
    _hint:["abort","abs","acos","apply","as_const","asin","atan","atan2","calloc","ceil","cerr","cin","clog","cos","cosh","cout","declval","endl","exchange","exit","exp","fabs","floor","fmod","forward","fprintf","fputs","free","frexp","fscanf","future","invoke","isalnum","isalpha","iscntrl","isdigit","isgraph","islower","isprint","ispunct","isspace","isupper","isxdigit","labs","launder","ldexp","log","log10","make_pair","make_shared","make_shared_for_overwrite","make_tuple","make_unique","malloc","memchr","memcmp","memcpy","memset","modf","move","pow","printf","putchar","puts","realloc","scanf","sin","sinh","snprintf","sprintf","sqrt","sscanf","std","stderr","stdin","stdout","strcat","strchr","strcmp","strcpy","strcspn","strlen","strncat","strncmp","strncpy","strpbrk","strrchr","strspn","strstr","swap","tan","tanh","terminate","to_underlying","tolower","toupper","vfprintf","visit","vprintf","vsprintf"]
    },
    begin:t(/\b/,/(?!decltype)/,/(?!if)/,/(?!for)/,/(?!while)/,n.IDENT_RE,(m=/(<[^<>]+>|)\s*\(/,
    t("(?=",m,")")))};var m;const g=[_,l,s,a,n.C_BLOCK_COMMENT_MODE,o,c],f={
    variants:[{begin:/=/,end:/;/},{begin:/\(/,end:/\)/},{
    beginKeywords:"new throw return else",end:/;/}],keywords:p,contains:g.concat([{
    begin:/\(/,end:/\)/,keywords:p,contains:g.concat(["self"]),relevance:0}]),
    relevance:0},b={className:"function",begin:"("+i+"[\\*&\\s]+)+"+d,
    returnBegin:!0,end:/[{;=]/,excludeEnd:!0,keywords:p,illegal:/[^\w\s\*&:<>.]/,
    contains:[{begin:"decltype\\(auto\\)",keywords:p,relevance:0},{begin:d,
    returnBegin:!0,contains:[u],relevance:0},{begin:/::/,relevance:0},{begin:/:/,
    endsWithParent:!0,contains:[c,o]},{relevance:0,match:/,/},{className:"params",
    begin:/\(/,end:/\)/,keywords:p,relevance:0,
    contains:[a,n.C_BLOCK_COMMENT_MODE,c,o,s,{begin:/\(/,end:/\)/,keywords:p,
    relevance:0,contains:["self",a,n.C_BLOCK_COMMENT_MODE,c,o,s]}]
    },s,a,n.C_BLOCK_COMMENT_MODE,l]};return{name:"C++",
    aliases:["cc","c++","h++","hpp","hh","hxx","cxx"],keywords:p,illegal:"</",
    classNameAliases:{"function.dispatch":"built_in"},
    contains:[].concat(f,b,_,g,[l,{
    begin:"\\b(deque|list|queue|priority_queue|pair|stack|vector|map|set|bitset|multiset|multimap|unordered_map|unordered_set|unordered_multiset|unordered_multimap|array|tuple|optional|variant|function)\\s*<",
    end:">",keywords:p,contains:["self",s]},{begin:n.IDENT_RE+"::",keywords:p},{
    match:[/\b(?:enum(?:\s+(?:class|struct))?|class|struct|union)/,/\s+/,/\w+/],
    className:{1:"keyword",3:"title.class"}}])}}})());hljs.registerLanguage("lua",(()=>{"use strict";return e=>{
    const t="\\[=*\\[",a="\\]=*\\]",n={begin:t,end:a,contains:["self"]
    },o=[e.COMMENT("--(?!\\[=*\\[)","$"),e.COMMENT("--\\[=*\\[",a,{contains:[n],
    relevance:10})];return{name:"Lua",keywords:{$pattern:e.UNDERSCORE_IDENT_RE,
    literal:"true false nil",
    keyword:"and break do else elseif end for goto if in local not or repeat return then until while",
    built_in:"_G _ENV _VERSION __index __newindex __mode __call __metatable __tostring __len __gc __add __sub __mul __div __mod __pow __concat __unm __eq __lt __le assert collectgarbage dofile error getfenv getmetatable ipairs load loadfile loadstring module next pairs pcall print rawequal rawget rawset require select setfenv setmetatable tonumber tostring type unpack xpcall arg self coroutine resume yield status wrap create running debug getupvalue debug sethook getmetatable gethook setmetatable setlocal traceback setfenv getinfo setupvalue getlocal getregistry getfenv io lines write close flush open output type read stderr stdin input stdout popen tmpfile math log max acos huge ldexp pi cos tanh pow deg tan cosh sinh random randomseed frexp ceil floor rad abs sqrt modf asin min mod fmod log10 atan2 exp sin atan os exit setlocale date getenv difftime remove time clock tmpname rename execute package preload loadlib loaded loaders cpath config path seeall string sub upper len gfind rep find match char dump gmatch reverse byte format gsub lower table setn insert getn foreachi maxn foreach concat sort remove"
    },contains:o.concat([{className:"function",beginKeywords:"function",end:"\\)",
    contains:[e.inherit(e.TITLE_MODE,{
    begin:"([_a-zA-Z]\\w*\\.)*([_a-zA-Z]\\w*:)?[_a-zA-Z]\\w*"}),{className:"params",
    begin:"\\(",endsWithParent:!0,contains:o}].concat(o)
    },e.C_NUMBER_MODE,e.APOS_STRING_MODE,e.QUOTE_STRING_MODE,{className:"string",
    begin:t,end:a,contains:[n],relevance:5}])}}})());hljs.registerLanguage("diff",(()=>{"use strict";function e(...e){
    return"("+((e=>{const n=e[e.length-1]
    ;return"object"==typeof n&&n.constructor===Object?(e.splice(e.length-1,1),n):{}
    })(e).capture?"":"?:")+e.map((e=>{return(n=e)?"string"==typeof n?n:n.source:null
    ;var n})).join("|")+")"}return n=>({name:"Diff",aliases:["patch"],contains:[{
    className:"meta",relevance:10,
    match:e(/^@@ +-\d+,\d+ +\+\d+,\d+ +@@/,/^\*\*\* +\d+,\d+ +\*\*\*\*$/,/^--- +\d+,\d+ +----$/)
    },{className:"comment",variants:[{
    begin:e(/Index: /,/^index/,/={3,}/,/^-{3}/,/^\*{3} /,/^\+{3}/,/^diff --git/),
    end:/$/},{match:/^\*{15}$/}]},{className:"addition",begin:/^\+/,end:/$/},{
    className:"deletion",begin:/^-/,end:/$/},{className:"addition",begin:/^!/,
    end:/$/}]})})());hljs.registerLanguage("crystal",(()=>{"use strict";return e=>{
    const n="(_?[ui](8|16|32|64|128))?",i="[a-zA-Z_]\\w*[!?=]?|[-+~]@|<<|>>|[=!]~|===?|<=>|[<>]=?|\\*\\*|[-/+%^&*~|]|//|//=|&[-+*]=?|&\\*\\*|\\[\\][=?]?",s="[A-Za-z_]\\w*(::\\w+)*(\\?|!)?",a={
    $pattern:"[a-zA-Z_]\\w*[!?=]?",
    keyword:"abstract alias annotation as as? asm begin break case class def do else elsif end ensure enum extend for fun if include instance_sizeof is_a? lib macro module next nil? of out pointerof private protected rescue responds_to? return require select self sizeof struct super then type typeof union uninitialized unless until verbatim when while with yield __DIR__ __END_LINE__ __FILE__ __LINE__",
    literal:"false nil true"},t={className:"subst",begin:/#\{/,end:/\}/,keywords:a
    },c={className:"template-variable",variants:[{begin:"\\{\\{",end:"\\}\\}"},{
    begin:"\\{%",end:"%\\}"}],keywords:a};function r(e,n){const i=[{begin:e,end:n}]
    ;return i[0].contains=i,i}const l={className:"string",
    contains:[e.BACKSLASH_ESCAPE,t],variants:[{begin:/'/,end:/'/},{begin:/"/,end:/"/
    },{begin:/`/,end:/`/},{begin:"%[Qwi]?\\(",end:"\\)",contains:r("\\(","\\)")},{
    begin:"%[Qwi]?\\[",end:"\\]",contains:r("\\[","\\]")},{begin:"%[Qwi]?\\{",
    end:/\}/,contains:r(/\{/,/\}/)},{begin:"%[Qwi]?<",end:">",contains:r("<",">")},{
    begin:"%[Qwi]?\\|",end:"\\|"},{begin:/<<-\w+$/,end:/^\s*\w+$/}],relevance:0},b={
    className:"string",variants:[{begin:"%q\\(",end:"\\)",contains:r("\\(","\\)")},{
    begin:"%q\\[",end:"\\]",contains:r("\\[","\\]")},{begin:"%q\\{",end:/\}/,
    contains:r(/\{/,/\}/)},{begin:"%q<",end:">",contains:r("<",">")},{begin:"%q\\|",
    end:"\\|"},{begin:/<<-'\w+'$/,end:/^\s*\w+$/}],relevance:0},o={
    begin:"(?!%\\})("+e.RE_STARTERS_RE+"|\\n|\\b(case|if|select|unless|until|when|while)\\b)\\s*",
    keywords:"case if select unless until when while",contains:[{className:"regexp",
    contains:[e.BACKSLASH_ESCAPE,t],variants:[{begin:"//[a-z]*",relevance:0},{
    begin:"/(?!\\/)",end:"/[a-z]*"}]}],relevance:0},g=[c,l,b,{className:"regexp",
    contains:[e.BACKSLASH_ESCAPE,t],variants:[{begin:"%r\\(",end:"\\)",
    contains:r("\\(","\\)")},{begin:"%r\\[",end:"\\]",contains:r("\\[","\\]")},{
    begin:"%r\\{",end:/\}/,contains:r(/\{/,/\}/)},{begin:"%r<",end:">",
    contains:r("<",">")},{begin:"%r\\|",end:"\\|"}],relevance:0},o,{
    className:"meta",begin:"@\\[",end:"\\]",
    contains:[e.inherit(e.QUOTE_STRING_MODE,{className:"string"})]},{
    className:"variable",
    begin:"(\\$\\W)|((\\$|@@?)(\\w+))(?=[^@$?])(?![A-Za-z])(?![@$?'])"
    },e.HASH_COMMENT_MODE,{className:"class",beginKeywords:"class module struct",
    end:"$|;",illegal:/=/,contains:[e.HASH_COMMENT_MODE,e.inherit(e.TITLE_MODE,{
    begin:s}),{begin:"<"}]},{className:"class",beginKeywords:"lib enum union",
    end:"$|;",illegal:/=/,contains:[e.HASH_COMMENT_MODE,e.inherit(e.TITLE_MODE,{
    begin:s})]},{beginKeywords:"annotation",end:"$|;",illegal:/=/,
    contains:[e.HASH_COMMENT_MODE,e.inherit(e.TITLE_MODE,{begin:s})],relevance:2},{
    className:"function",beginKeywords:"def",end:/\B\b/,
    contains:[e.inherit(e.TITLE_MODE,{begin:i,endsParent:!0})]},{
    className:"function",beginKeywords:"fun macro",end:/\B\b/,
    contains:[e.inherit(e.TITLE_MODE,{begin:i,endsParent:!0})],relevance:2},{
    className:"symbol",begin:e.UNDERSCORE_IDENT_RE+"(!|\\?)?:",relevance:0},{
    className:"symbol",begin:":",contains:[l,{begin:i}],relevance:0},{
    className:"number",variants:[{begin:"\\b0b([01_]+)"+n},{begin:"\\b0o([0-7_]+)"+n
    },{begin:"\\b0x([A-Fa-f0-9_]+)"+n},{
    begin:"\\b([1-9][0-9_]*[0-9]|[0-9])(\\.[0-9][0-9_]*)?([eE]_?[-+]?[0-9_]*)?(_?f(32|64))?(?!_)"
    },{begin:"\\b([1-9][0-9_]*|0)"+n}],relevance:0}]
    ;return t.contains=g,c.contains=g.slice(1),{name:"Crystal",aliases:["cr"],
    keywords:a,contains:g}}})());hljs.registerLanguage("clojure",(()=>{"use strict";return e=>{
    const t="a-zA-Z_\\-!.?+*=<>&#'",n="["+t+"]["+t+"0-9/;:]*",r="def defonce defprotocol defstruct defmulti defmethod defn- defn defmacro deftype defrecord",a={
    $pattern:n,
    built_in:r+" cond apply if-not if-let if not not= =|0 <|0 >|0 <=|0 >=|0 ==|0 +|0 /|0 *|0 -|0 rem quot neg? pos? delay? symbol? keyword? true? false? integer? empty? coll? list? set? ifn? fn? associative? sequential? sorted? counted? reversible? number? decimal? class? distinct? isa? float? rational? reduced? ratio? odd? even? char? seq? vector? string? map? nil? contains? zero? instance? not-every? not-any? libspec? -> ->> .. . inc compare do dotimes mapcat take remove take-while drop letfn drop-last take-last drop-while while intern condp case reduced cycle split-at split-with repeat replicate iterate range merge zipmap declare line-seq sort comparator sort-by dorun doall nthnext nthrest partition eval doseq await await-for let agent atom send send-off release-pending-sends add-watch mapv filterv remove-watch agent-error restart-agent set-error-handler error-handler set-error-mode! error-mode shutdown-agents quote var fn loop recur throw try monitor-enter monitor-exit macroexpand macroexpand-1 for dosync and or when when-not when-let comp juxt partial sequence memoize constantly complement identity assert peek pop doto proxy first rest cons cast coll last butlast sigs reify second ffirst fnext nfirst nnext meta with-meta ns in-ns create-ns import refer keys select-keys vals key val rseq name namespace promise into transient persistent! conj! assoc! dissoc! pop! disj! use class type num float double short byte boolean bigint biginteger bigdec print-method print-dup throw-if printf format load compile get-in update-in pr pr-on newline flush read slurp read-line subvec with-open memfn time re-find re-groups rand-int rand mod locking assert-valid-fdecl alias resolve ref deref refset swap! reset! set-validator! compare-and-set! alter-meta! reset-meta! commute get-validator alter ref-set ref-history-count ref-min-history ref-max-history ensure sync io! new next conj set! to-array future future-call into-array aset gen-class reduce map filter find empty hash-map hash-set sorted-map sorted-map-by sorted-set sorted-set-by vec vector seq flatten reverse assoc dissoc list disj get union difference intersection extend extend-type extend-protocol int nth delay count concat chunk chunk-buffer chunk-append chunk-first chunk-rest max min dec unchecked-inc-int unchecked-inc unchecked-dec-inc unchecked-dec unchecked-negate unchecked-add-int unchecked-add unchecked-subtract-int unchecked-subtract chunk-next chunk-cons chunked-seq? prn vary-meta lazy-seq spread list* str find-keyword keyword symbol gensym force rationalize"
    },s={begin:n,relevance:0},o={className:"number",begin:"[-+]?\\d+(\\.\\d+)?",
    relevance:0},i=e.inherit(e.QUOTE_STRING_MODE,{illegal:null
    }),c=e.COMMENT(";","$",{relevance:0}),d={className:"literal",
    begin:/\b(true|false|nil)\b/},l={begin:"[\\[\\{]",end:"[\\]\\}]",relevance:0
    },m={className:"comment",begin:"\\^"+n},p=e.COMMENT("\\^\\{","\\}"),u={
    className:"symbol",begin:"[:]{1,2}"+n},f={begin:"\\(",end:"\\)"},h={
    endsWithParent:!0,relevance:0},y={keywords:a,className:"name",begin:n,
    relevance:0,starts:h},g=[f,i,m,p,c,u,l,o,d,s],b={beginKeywords:r,keywords:{
    $pattern:n,keyword:r},end:'(\\[|#|\\d|"|:|\\{|\\)|\\(|$)',contains:[{
    className:"title",begin:n,relevance:0,excludeEnd:!0,endsParent:!0}].concat(g)}
    ;return f.contains=[e.COMMENT("comment",""),b,y,h],
    h.contains=g,l.contains=g,p.contains=[l],{name:"Clojure",aliases:["clj"],
    illegal:/\S/,contains:[f,i,m,p,c,u,l,o,d]}}})());hljs.registerLanguage("vbnet",(()=>{"use strict";function e(e){
    return e?"string"==typeof e?e:e.source:null}function n(...n){
    return n.map((n=>e(n))).join("")}function t(...n){return"("+((e=>{
    const n=e[e.length-1]
    ;return"object"==typeof n&&n.constructor===Object?(e.splice(e.length-1,1),n):{}
    })(n).capture?"":"?:")+n.map((n=>e(n))).join("|")+")"}return e=>{
    const a=/\d{1,2}\/\d{1,2}\/\d{4}/,i=/\d{4}-\d{1,2}-\d{1,2}/,s=/(\d|1[012])(:\d+){0,2} *(AM|PM)/,r=/\d{1,2}(:\d{1,2}){1,2}/,o={
    className:"literal",variants:[{begin:n(/# */,t(i,a),/ *#/)},{
    begin:n(/# */,r,/ *#/)},{begin:n(/# */,s,/ *#/)},{
    begin:n(/# */,t(i,a),/ +/,t(s,r),/ *#/)}]},l=e.COMMENT(/'''/,/$/,{contains:[{
    className:"doctag",begin:/<\/?/,end:/>/}]}),c=e.COMMENT(null,/$/,{variants:[{
    begin:/'/},{begin:/([\t ]|^)REM(?=\s)/}]});return{name:"Visual Basic .NET",
    aliases:["vb"],case_insensitive:!0,classNameAliases:{label:"symbol"},keywords:{
    keyword:"addhandler alias aggregate ansi as async assembly auto binary by byref byval call case catch class compare const continue custom declare default delegate dim distinct do each equals else elseif end enum erase error event exit explicit finally for friend from function get global goto group handles if implements imports in inherits interface into iterator join key let lib loop me mid module mustinherit mustoverride mybase myclass namespace narrowing new next notinheritable notoverridable of off on operator option optional order overloads overridable overrides paramarray partial preserve private property protected public raiseevent readonly redim removehandler resume return select set shadows shared skip static step stop structure strict sub synclock take text then throw to try unicode until using when where while widening with withevents writeonly yield",
    built_in:"addressof and andalso await directcast gettype getxmlnamespace is isfalse isnot istrue like mod nameof new not or orelse trycast typeof xor cbool cbyte cchar cdate cdbl cdec cint clng cobj csbyte cshort csng cstr cuint culng cushort",
    type:"boolean byte char date decimal double integer long object sbyte short single string uinteger ulong ushort",
    literal:"true false nothing"},
    illegal:"//|\\{|\\}|endif|gosub|variant|wend|^\\$ ",contains:[{
    className:"string",begin:/"(""|[^/n])"C\b/},{className:"string",begin:/"/,
    end:/"/,illegal:/\n/,contains:[{begin:/""/}]},o,{className:"number",relevance:0,
    variants:[{begin:/\b\d[\d_]*((\.[\d_]+(E[+-]?[\d_]+)?)|(E[+-]?[\d_]+))[RFD@!#]?/
    },{begin:/\b\d[\d_]*((U?[SIL])|[%&])?/},{begin:/&H[\dA-F_]+((U?[SIL])|[%&])?/},{
    begin:/&O[0-7_]+((U?[SIL])|[%&])?/},{begin:/&B[01_]+((U?[SIL])|[%&])?/}]},{
    className:"label",begin:/^\w+:/},l,c,{className:"meta",
    begin:/[\t ]*#(const|disable|else|elseif|enable|end|externalsource|if|region)\b/,
    end:/$/,keywords:{
    keyword:"const disable else elseif enable end externalsource if region then"},
    contains:[c]}]}}})());hljs.registerLanguage("ruby",(()=>{"use strict";function e(e){
    return n("(?=",e,")")}function n(...e){return e.map((e=>{
    return(n=e)?"string"==typeof n?n:n.source:null;var n})).join("")}return a=>{
    const i="([a-zA-Z_]\\w*[!?=]?|[-+~]@|<<|>>|=~|===?|<=>|[<>]=?|\\*\\*|[-/+%^&*~`|]|\\[\\]=?)",s={
    keyword:"and then defined module in return redo if BEGIN retry end for self when next until do begin unless END rescue else break undef not super class case require yield alias while ensure elsif or include attr_reader attr_writer attr_accessor __FILE__",
    built_in:"proc lambda",literal:"true false nil"},r={className:"doctag",
    begin:"@[A-Za-z]+"},b={begin:"#<",end:">"},c=[a.COMMENT("#","$",{contains:[r]
    }),a.COMMENT("^=begin","^=end",{contains:[r],relevance:10
    }),a.COMMENT("^__END__","\\n$")],t={className:"subst",begin:/#\{/,end:/\}/,
    keywords:s},g={className:"string",contains:[a.BACKSLASH_ESCAPE,t],variants:[{
    begin:/'/,end:/'/},{begin:/"/,end:/"/},{begin:/`/,end:/`/},{begin:/%[qQwWx]?\(/,
    end:/\)/},{begin:/%[qQwWx]?\[/,end:/\]/},{begin:/%[qQwWx]?\{/,end:/\}/},{
    begin:/%[qQwWx]?</,end:/>/},{begin:/%[qQwWx]?\//,end:/\//},{begin:/%[qQwWx]?%/,
    end:/%/},{begin:/%[qQwWx]?-/,end:/-/},{begin:/%[qQwWx]?\|/,end:/\|/},{
    begin:/\B\?(\\\d{1,3})/},{begin:/\B\?(\\x[A-Fa-f0-9]{1,2})/},{
    begin:/\B\?(\\u\{?[A-Fa-f0-9]{1,6}\}?)/},{
    begin:/\B\?(\\M-\\C-|\\M-\\c|\\c\\M-|\\M-|\\C-\\M-)[\x20-\x7e]/},{
    begin:/\B\?\\(c|C-)[\x20-\x7e]/},{begin:/\B\?\\?\S/},{
    begin:n(/<<[-~]?'?/,e(/(\w+)(?=\W)[^\n]*\n(?:[^\n]*\n)*?\s*\1\b/)),
    contains:[a.END_SAME_AS_BEGIN({begin:/(\w+)/,end:/(\w+)/,
    contains:[a.BACKSLASH_ESCAPE,t]})]}]},d="[0-9](_?[0-9])*",l={className:"number",
    relevance:0,variants:[{
    begin:`\\b([1-9](_?[0-9])*|0)(\\.(${d}))?([eE][+-]?(${d})|r)?i?\\b`},{
    begin:"\\b0[dD][0-9](_?[0-9])*r?i?\\b"},{begin:"\\b0[bB][0-1](_?[0-1])*r?i?\\b"
    },{begin:"\\b0[oO][0-7](_?[0-7])*r?i?\\b"},{
    begin:"\\b0[xX][0-9a-fA-F](_?[0-9a-fA-F])*r?i?\\b"},{
    begin:"\\b0(_?[0-7])+r?i?\\b"}]},o={className:"params",begin:"\\(",end:"\\)",
    endsParent:!0,keywords:s},_=[g,{className:"class",beginKeywords:"class module",
    end:"$|;",illegal:/=/,contains:[a.inherit(a.TITLE_MODE,{
    begin:"[A-Za-z_]\\w*(::\\w+)*(\\?|!)?"}),{begin:"<\\s*",contains:[{
    begin:"("+a.IDENT_RE+"::)?"+a.IDENT_RE,relevance:0}]}].concat(c)},{
    className:"function",begin:n(/def\s+/,e(i+"\\s*(\\(|;|$)")),relevance:0,
    keywords:"def",end:"$|;",contains:[a.inherit(a.TITLE_MODE,{begin:i
    }),o].concat(c)},{begin:a.IDENT_RE+"::"},{className:"symbol",
    begin:a.UNDERSCORE_IDENT_RE+"(!|\\?)?:",relevance:0},{className:"symbol",
    begin:":(?!\\s)",contains:[g,{begin:i}],relevance:0},l,{className:"variable",
    begin:"(\\$\\W)|((\\$|@@?)(\\w+))(?=[^@$?])(?![A-Za-z])(?![@$?'])"},{
    className:"params",begin:/\|/,end:/\|/,relevance:0,keywords:s},{
    begin:"("+a.RE_STARTERS_RE+"|unless)\\s*",keywords:"unless",contains:[{
    className:"regexp",contains:[a.BACKSLASH_ESCAPE,t],illegal:/\n/,variants:[{
    begin:"/",end:"/[a-z]*"},{begin:/%r\{/,end:/\}[a-z]*/},{begin:"%r\\(",
    end:"\\)[a-z]*"},{begin:"%r!",end:"![a-z]*"},{begin:"%r\\[",end:"\\][a-z]*"}]
    }].concat(b,c),relevance:0}].concat(b,c);t.contains=_,o.contains=_;const E=[{
    begin:/^\s*=>/,starts:{end:"$",contains:_}},{className:"meta",
    begin:"^([>?]>|[\\w#]+\\(\\w+\\):\\d+:\\d+>|(\\w+-)?\\d+\\.\\d+\\.\\d+(p\\d+)?[^\\d][^>]+>)(?=[ ])",
    starts:{end:"$",contains:_}}];return c.unshift(b),{name:"Ruby",
    aliases:["rb","gemspec","podspec","thor","irb"],keywords:s,illegal:/\/\*/,
    contains:[a.SHEBANG({binary:"ruby"})].concat(E).concat(c).concat(_)}}})());hljs.registerLanguage("coffeescript",(()=>{"use strict"
    ;const e=["as","in","of","if","for","while","finally","var","new","function","do","return","void","else","break","catch","instanceof","with","throw","case","default","try","switch","continue","typeof","delete","let","yield","const","class","debugger","async","await","static","import","from","export","extends"],n=["true","false","null","undefined","NaN","Infinity"],r=[].concat(["setInterval","setTimeout","clearInterval","clearTimeout","require","exports","eval","isFinite","isNaN","parseFloat","parseInt","decodeURI","decodeURIComponent","encodeURI","encodeURIComponent","escape","unescape"],["Intl","DataView","Number","Math","Date","String","RegExp","Object","Function","Boolean","Error","Symbol","Set","Map","WeakSet","WeakMap","Proxy","Reflect","JSON","Promise","Float64Array","Int16Array","Int32Array","Int8Array","Uint16Array","Uint32Array","Float32Array","Array","Uint8Array","Uint8ClampedArray","ArrayBuffer","BigInt64Array","BigUint64Array","BigInt"],["EvalError","InternalError","RangeError","ReferenceError","SyntaxError","TypeError","URIError"])
    ;return a=>{const t={
    keyword:e.concat(["then","unless","until","loop","by","when","and","or","is","isnt","not"]).filter((i=["var","const","let","function","static"],
    e=>!i.includes(e))),literal:n.concat(["yes","no","on","off"]),
    built_in:r.concat(["npm","print"])};var i;const s="[A-Za-z$_][0-9A-Za-z$_]*",o={
    className:"subst",begin:/#\{/,end:/\}/,keywords:t
    },c=[a.BINARY_NUMBER_MODE,a.inherit(a.C_NUMBER_MODE,{starts:{end:"(\\s*/)?",
    relevance:0}}),{className:"string",variants:[{begin:/'''/,end:/'''/,
    contains:[a.BACKSLASH_ESCAPE]},{begin:/'/,end:/'/,contains:[a.BACKSLASH_ESCAPE]
    },{begin:/"""/,end:/"""/,contains:[a.BACKSLASH_ESCAPE,o]},{begin:/"/,end:/"/,
    contains:[a.BACKSLASH_ESCAPE,o]}]},{className:"regexp",variants:[{begin:"///",
    end:"///",contains:[o,a.HASH_COMMENT_MODE]},{begin:"//[gim]{0,3}(?=\\W)",
    relevance:0},{begin:/\/(?![ *]).*?(?![\\]).\/[gim]{0,3}(?=\W)/}]},{begin:"@"+s
    },{subLanguage:"javascript",excludeBegin:!0,excludeEnd:!0,variants:[{
    begin:"```",end:"```"},{begin:"`",end:"`"}]}];o.contains=c
    ;const l=a.inherit(a.TITLE_MODE,{begin:s}),d="(\\(.*\\)\\s*)?\\B[-=]>",g={
    className:"params",begin:"\\([^\\(]",returnBegin:!0,contains:[{begin:/\(/,
    end:/\)/,keywords:t,contains:["self"].concat(c)}]};return{name:"CoffeeScript",
    aliases:["coffee","cson","iced"],keywords:t,illegal:/\/\*/,
    contains:[...c,a.COMMENT("###","###"),a.HASH_COMMENT_MODE,{className:"function",
    begin:"^\\s*"+s+"\\s*=\\s*"+d,end:"[-=]>",returnBegin:!0,contains:[l,g]},{
    begin:/[:\(,=]\s*/,relevance:0,contains:[{className:"function",begin:d,
    end:"[-=]>",returnBegin:!0,contains:[g]}]},{className:"class",
    beginKeywords:"class",end:"$",illegal:/[:="\[\]]/,contains:[{
    beginKeywords:"extends",endsWithParent:!0,illegal:/[:="\[\]]/,contains:[l]},l]
    },{begin:s+":",end:":",returnBegin:!0,returnEnd:!0,relevance:0}]}}})());hljs.registerLanguage("clojure-repl",(()=>{"use strict";return e=>({
    name:"Clojure REPL",contains:[{className:"meta",begin:/^([\w.-]+|\s*#_)?=>/,
    starts:{end:/$/,subLanguage:"clojure"}}]})})());hljs.registerLanguage("erlang",(()=>{"use strict";return e=>{
    const n="[a-z'][a-zA-Z0-9_']*",r="("+n+":"+n+"|"+n+")",a={
    keyword:"after and andalso|10 band begin bnot bor bsl bzr bxor case catch cond div end fun if let not of orelse|10 query receive rem try when xor",
    literal:"false true"},i=e.COMMENT("%","$"),s={className:"number",
    begin:"\\b(\\d+(_\\d+)*#[a-fA-F0-9]+(_[a-fA-F0-9]+)*|\\d+(_\\d+)*(\\.\\d+(_\\d+)*)?([eE][-+]?\\d+)?)",
    relevance:0},c={begin:"fun\\s+"+n+"/\\d+"},t={begin:r+"\\(",end:"\\)",
    returnBegin:!0,relevance:0,contains:[{begin:r,relevance:0},{begin:"\\(",
    end:"\\)",endsWithParent:!0,returnEnd:!0,relevance:0}]},d={begin:/\{/,end:/\}/,
    relevance:0},o={begin:"\\b_([A-Z][A-Za-z0-9_]*)?",relevance:0},l={
    begin:"[A-Z][a-zA-Z0-9_]*",relevance:0},b={begin:"#"+e.UNDERSCORE_IDENT_RE,
    relevance:0,returnBegin:!0,contains:[{begin:"#"+e.UNDERSCORE_IDENT_RE,
    relevance:0},{begin:/\{/,end:/\}/,relevance:0}]},g={
    beginKeywords:"fun receive if try case",end:"end",keywords:a}
    ;g.contains=[i,c,e.inherit(e.APOS_STRING_MODE,{className:""
    }),g,t,e.QUOTE_STRING_MODE,s,d,o,l,b]
    ;const E=[i,c,g,t,e.QUOTE_STRING_MODE,s,d,o,l,b]
    ;t.contains[1].contains=E,d.contains=E,b.contains[1].contains=E;const u={
    className:"params",begin:"\\(",end:"\\)",contains:E};return{name:"Erlang",
    aliases:["erl"],keywords:a,illegal:"(</|\\*=|\\+=|-=|/\\*|\\*/|\\(\\*|\\*\\))",
    contains:[{className:"function",begin:"^"+n+"\\s*\\(",end:"->",returnBegin:!0,
    illegal:"\\(|#|//|/\\*|\\\\|:|;",contains:[u,e.inherit(e.TITLE_MODE,{begin:n})],
    starts:{end:";|\\.",keywords:a,contains:E}},i,{begin:"^-",end:"\\.",relevance:0,
    excludeEnd:!0,returnBegin:!0,keywords:{$pattern:"-"+e.IDENT_RE,
    keyword:["-module","-record","-undef","-export","-ifdef","-ifndef","-author","-copyright","-doc","-vsn","-import","-include","-include_lib","-compile","-define","-else","-endif","-file","-behaviour","-behavior","-spec"].map((e=>e+"|1.5")).join(" ")
    },contains:[u]},s,e.QUOTE_STRING_MODE,b,o,l,d,{begin:/\.$/}]}}})());hljs.registerLanguage("java",(()=>{"use strict"
    ;var e="\\.([0-9](_*[0-9])*)",a="[0-9a-fA-F](_*[0-9a-fA-F])*",n={
    className:"number",variants:[{
    begin:`(\\b([0-9](_*[0-9])*)((${e})|\\.)?|(${e}))[eE][+-]?([0-9](_*[0-9])*)[fFdD]?\\b`
    },{begin:`\\b([0-9](_*[0-9])*)((${e})[fFdD]?\\b|\\.([fFdD]\\b)?)`},{
    begin:`(${e})[fFdD]?\\b`},{begin:"\\b([0-9](_*[0-9])*)[fFdD]\\b"},{
    begin:`\\b0[xX]((${a})\\.?|(${a})?\\.(${a}))[pP][+-]?([0-9](_*[0-9])*)[fFdD]?\\b`
    },{begin:"\\b(0|[1-9](_*[0-9])*)[lL]?\\b"},{begin:`\\b0[xX](${a})[lL]?\\b`},{
    begin:"\\b0(_*[0-7])*[lL]?\\b"},{begin:"\\b0[bB][01](_*[01])*[lL]?\\b"}],
    relevance:0};function s(e,a,n){return-1===n?"":e.replace(a,(t=>s(e,a,n-1)))}
    return e=>{
    const a="[\xc0-\u02b8a-zA-Z_$][\xc0-\u02b8a-zA-Z_$0-9]*",t=a+s("(?:<"+a+"~~~(?:\\s*,\\s*"+a+"~~~)*>)?",/~~~/g,2),i={
    keyword:["synchronized","abstract","private","var","static","if","const ","for","while","strictfp","finally","protected","import","native","final","void","enum","else","break","transient","catch","instanceof","volatile","case","assert","package","default","public","try","switch","continue","throws","protected","public","private","module","requires","exports","do"],
    literal:["false","true","null"],
    type:["char","boolean","long","float","int","byte","short","double"],
    built_in:["super","this"]},r={className:"meta",begin:"@"+a,contains:[{
    begin:/\(/,end:/\)/,contains:["self"]}]},l={className:"params",begin:/\(/,
    end:/\)/,keywords:i,relevance:0,contains:[e.C_BLOCK_COMMENT_MODE],endsParent:!0}
    ;return{name:"Java",aliases:["jsp"],keywords:i,illegal:/<\/|#/,
    contains:[e.COMMENT("/\\*\\*","\\*/",{relevance:0,contains:[{begin:/\w+@/,
    relevance:0},{className:"doctag",begin:"@[A-Za-z]+"}]}),{
    begin:/import java\.[a-z]+\./,keywords:"import",relevance:2
    },e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE,e.APOS_STRING_MODE,e.QUOTE_STRING_MODE,{
    match:[/\b(?:class|interface|enum|extends|implements|new)/,/\s+/,a],className:{
    1:"keyword",3:"title.class"}},{begin:[a,/\s+/,a,/\s+/,/=/],className:{1:"type",
    3:"variable",5:"operator"}},{begin:[/record/,/\s+/,a],className:{1:"keyword",
    3:"title.class"},contains:[l,e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE]},{
    beginKeywords:"new throw return else",relevance:0},{
    begin:["(?:"+t+"\\s+)",e.UNDERSCORE_IDENT_RE,/\s*(?=\()/],className:{
    2:"title.function"},keywords:i,contains:[{className:"params",begin:/\(/,
    end:/\)/,keywords:i,relevance:0,
    contains:[r,e.APOS_STRING_MODE,e.QUOTE_STRING_MODE,n,e.C_BLOCK_COMMENT_MODE]
    },e.C_LINE_COMMENT_MODE,e.C_BLOCK_COMMENT_MODE]},n,r]}}})());hljs.registerLanguage("javascript",(()=>{"use strict"
    ;const e="[A-Za-z$_][0-9A-Za-z$_]*",n=["as","in","of","if","for","while","finally","var","new","function","do","return","void","else","break","catch","instanceof","with","throw","case","default","try","switch","continue","typeof","delete","let","yield","const","class","debugger","async","await","static","import","from","export","extends"],a=["true","false","null","undefined","NaN","Infinity"],t=["Intl","DataView","Number","Math","Date","String","RegExp","Object","Function","Boolean","Error","Symbol","Set","Map","WeakSet","WeakMap","Proxy","Reflect","JSON","Promise","Float64Array","Int16Array","Int32Array","Int8Array","Uint16Array","Uint32Array","Float32Array","Array","Uint8Array","Uint8ClampedArray","ArrayBuffer","BigInt64Array","BigUint64Array","BigInt"],s=["EvalError","InternalError","RangeError","ReferenceError","SyntaxError","TypeError","URIError"],r=["setInterval","setTimeout","clearInterval","clearTimeout","require","exports","eval","isFinite","isNaN","parseFloat","parseInt","decodeURI","decodeURIComponent","encodeURI","encodeURIComponent","escape","unescape"],i=["arguments","this","super","console","window","document","localStorage","module","global"],c=[].concat(r,t,s)
    ;function o(e){return l("(?=",e,")")}function l(...e){return e.map((e=>{
    return(n=e)?"string"==typeof n?n:n.source:null;var n})).join("")}return b=>{
    const g=e,d={begin:/<[A-Za-z0-9\\._:-]+/,end:/\/[A-Za-z0-9\\._:-]+>|\/>/,
    isTrulyOpeningTag:(e,n)=>{const a=e[0].length+e.index,t=e.input[a]
    ;"<"!==t?">"===t&&(((e,{after:n})=>{const a="</"+e[0].slice(1)
    ;return-1!==e.input.indexOf(a,n)})(e,{after:a
    })||n.ignoreMatch()):n.ignoreMatch()}},u={$pattern:e,keyword:n,literal:a,
    built_in:c,"variable.language":i
    },m="\\.([0-9](_?[0-9])*)",E="0|[1-9](_?[0-9])*|0[0-7]*[89][0-9]*",y={
    className:"number",variants:[{
    begin:`(\\b(${E})((${m})|\\.)?|(${m}))[eE][+-]?([0-9](_?[0-9])*)\\b`},{
    begin:`\\b(${E})\\b((${m})\\b|\\.)?|(${m})\\b`},{
    begin:"\\b(0|[1-9](_?[0-9])*)n\\b"},{
    begin:"\\b0[xX][0-9a-fA-F](_?[0-9a-fA-F])*n?\\b"},{
    begin:"\\b0[bB][0-1](_?[0-1])*n?\\b"},{begin:"\\b0[oO][0-7](_?[0-7])*n?\\b"},{
    begin:"\\b0[0-7]+n?\\b"}],relevance:0},_={className:"subst",begin:"\\$\\{",
    end:"\\}",keywords:u,contains:[]},N={begin:"html`",end:"",starts:{end:"`",
    returnEnd:!1,contains:[b.BACKSLASH_ESCAPE,_],subLanguage:"xml"}},f={
    begin:"css`",end:"",starts:{end:"`",returnEnd:!1,
    contains:[b.BACKSLASH_ESCAPE,_],subLanguage:"css"}},A={className:"string",
    begin:"`",end:"`",contains:[b.BACKSLASH_ESCAPE,_]},v={className:"comment",
    variants:[b.COMMENT(/\/\*\*(?!\/)/,"\\*/",{relevance:0,contains:[{
    begin:"(?=@[A-Za-z]+)",relevance:0,contains:[{className:"doctag",
    begin:"@[A-Za-z]+"},{className:"type",begin:"\\{",end:"\\}",excludeEnd:!0,
    excludeBegin:!0,relevance:0},{className:"variable",begin:g+"(?=\\s*(-)|$)",
    endsParent:!0,relevance:0},{begin:/(?=[^\n])\s/,relevance:0}]}]
    }),b.C_BLOCK_COMMENT_MODE,b.C_LINE_COMMENT_MODE]
    },p=[b.APOS_STRING_MODE,b.QUOTE_STRING_MODE,N,f,A,y,b.REGEXP_MODE]
    ;_.contains=p.concat({begin:/\{/,end:/\}/,keywords:u,contains:["self"].concat(p)
    });const h=[].concat(v,_.contains),S=h.concat([{begin:/\(/,end:/\)/,keywords:u,
    contains:["self"].concat(h)}]),w={className:"params",begin:/\(/,end:/\)/,
    excludeBegin:!0,excludeEnd:!0,keywords:u,contains:S},R={variants:[{
    match:[/class/,/\s+/,g],scope:{1:"keyword",3:"title.class"}},{
    match:[/extends/,/\s+/,l(g,"(",l(/\./,g),")*")],scope:{1:"keyword",
    3:"title.class.inherited"}}]},O={relevance:0,
    match:/\b[A-Z][a-z]+([A-Z][a-z]+)*/,className:"title.class",keywords:{
    _:[...t,...s]}},I={variants:[{match:[/function/,/\s+/,g,/(?=\s*\()/]},{
    match:[/function/,/\s*(?=\()/]}],className:{1:"keyword",3:"title.function"},
    label:"func.def",contains:[w],illegal:/%/},T={
    match:l(/\b/,(x=[...r,"super"],l("(?!",x.join("|"),")")),g,o(/\(/)),
    className:"title.function",relevance:0};var x;const M={
    begin:l(/\./,o(l(g,/(?![0-9A-Za-z$_(])/))),end:g,excludeBegin:!0,
    keywords:"prototype",className:"property",relevance:0},k={
    match:[/get|set/,/\s+/,g,/(?=\()/],className:{1:"keyword",3:"title.function"},
    contains:[{begin:/\(\)/},w]
    },C="(\\([^()]*(\\([^()]*(\\([^()]*\\)[^()]*)*\\)[^()]*)*\\)|"+b.UNDERSCORE_IDENT_RE+")\\s*=>",B={
    match:[/const|var|let/,/\s+/,g,/\s*/,/=\s*/,o(C)],className:{1:"keyword",
    3:"title.function"},contains:[w]};return{name:"Javascript",
    aliases:["js","jsx","mjs","cjs"],keywords:u,exports:{PARAMS_CONTAINS:S},
    illegal:/#(?![$_A-z])/,contains:[b.SHEBANG({label:"shebang",binary:"node",
    relevance:5}),{label:"use_strict",className:"meta",relevance:10,
    begin:/^\s*['"]use (strict|asm)['"]/
    },b.APOS_STRING_MODE,b.QUOTE_STRING_MODE,N,f,A,v,y,O,{className:"attr",
    begin:g+o(":"),relevance:0},B,{
    begin:"("+b.RE_STARTERS_RE+"|\\b(case|return|throw)\\b)\\s*",
    keywords:"return throw case",relevance:0,contains:[v,b.REGEXP_MODE,{
    className:"function",begin:C,returnBegin:!0,end:"\\s*=>",contains:[{
    className:"params",variants:[{begin:b.UNDERSCORE_IDENT_RE,relevance:0},{
    className:null,begin:/\(\s*\)/,skip:!0},{begin:/\(/,end:/\)/,excludeBegin:!0,
    excludeEnd:!0,keywords:u,contains:S}]}]},{begin:/,/,relevance:0},{match:/\s+/,
    relevance:0},{variants:[{begin:"<>",end:"</>"},{begin:d.begin,
    "on:begin":d.isTrulyOpeningTag,end:d.end}],subLanguage:"xml",contains:[{
    begin:d.begin,end:d.end,skip:!0,contains:["self"]}]}]},I,{
    beginKeywords:"while if switch catch for"},{
    begin:"\\b(?!function)"+b.UNDERSCORE_IDENT_RE+"\\([^()]*(\\([^()]*(\\([^()]*\\)[^()]*)*\\)[^()]*)*\\)\\s*\\{",
    returnBegin:!0,label:"func.def",contains:[w,b.inherit(b.TITLE_MODE,{begin:g,
    className:"title.function"})]},{match:/\.\.\./,relevance:0},M,{match:"\\$"+g,
    relevance:0},{match:[/\bconstructor(?=\s*\()/],className:{1:"title.function"},
    contains:[w]},T,{relevance:0,match:/\b[A-Z][A-Z_]+\b/,
    className:"variable.constant"},R,k,{match:/\$[(.]/}]}}})());hljs.registerLanguage("swift",(()=>{"use strict";function e(e){
    return e?"string"==typeof e?e:e.source:null}function a(e){return t("(?=",e,")")}
    function t(...a){return a.map((a=>e(a))).join("")}function n(...a){
    return"("+((e=>{const a=e[e.length-1]
    ;return"object"==typeof a&&a.constructor===Object?(e.splice(e.length-1,1),a):{}
    })(a).capture?"":"?:")+a.map((a=>e(a))).join("|")+")"}
    const i=e=>t(/\b/,e,/\w$/.test(e)?/\b/:/\B/),s=["Protocol","Type"].map(i),u=["init","self"].map(i),c=["Any","Self"],r=["actor","associatedtype","async","await",/as\?/,/as!/,"as","break","case","catch","class","continue","convenience","default","defer","deinit","didSet","do","dynamic","else","enum","extension","fallthrough",/fileprivate\(set\)/,"fileprivate","final","for","func","get","guard","if","import","indirect","infix",/init\?/,/init!/,"inout",/internal\(set\)/,"internal","in","is","lazy","let","mutating","nonmutating",/open\(set\)/,"open","operator","optional","override","postfix","precedencegroup","prefix",/private\(set\)/,"private","protocol",/public\(set\)/,"public","repeat","required","rethrows","return","set","some","static","struct","subscript","super","switch","throws","throw",/try\?/,/try!/,"try","typealias",/unowned\(safe\)/,/unowned\(unsafe\)/,"unowned","var","weak","where","while","willSet"],o=["false","nil","true"],l=["assignment","associativity","higherThan","left","lowerThan","none","right"],m=["#colorLiteral","#column","#dsohandle","#else","#elseif","#endif","#error","#file","#fileID","#fileLiteral","#filePath","#function","#if","#imageLiteral","#keyPath","#line","#selector","#sourceLocation","#warn_unqualified_access","#warning"],p=["abs","all","any","assert","assertionFailure","debugPrint","dump","fatalError","getVaList","isKnownUniquelyReferenced","max","min","numericCast","pointwiseMax","pointwiseMin","precondition","preconditionFailure","print","readLine","repeatElement","sequence","stride","swap","swift_unboxFromSwiftValueWithType","transcode","type","unsafeBitCast","unsafeDowncast","withExtendedLifetime","withUnsafeMutablePointer","withUnsafePointer","withVaList","withoutActuallyEscaping","zip"],F=n(/[/=\-+!*%<>&|^~?]/,/[\u00A1-\u00A7]/,/[\u00A9\u00AB]/,/[\u00AC\u00AE]/,/[\u00B0\u00B1]/,/[\u00B6\u00BB\u00BF\u00D7\u00F7]/,/[\u2016-\u2017]/,/[\u2020-\u2027]/,/[\u2030-\u203E]/,/[\u2041-\u2053]/,/[\u2055-\u205E]/,/[\u2190-\u23FF]/,/[\u2500-\u2775]/,/[\u2794-\u2BFF]/,/[\u2E00-\u2E7F]/,/[\u3001-\u3003]/,/[\u3008-\u3020]/,/[\u3030]/),d=n(F,/[\u0300-\u036F]/,/[\u1DC0-\u1DFF]/,/[\u20D0-\u20FF]/,/[\uFE00-\uFE0F]/,/[\uFE20-\uFE2F]/),b=t(F,d,"*"),h=n(/[a-zA-Z_]/,/[\u00A8\u00AA\u00AD\u00AF\u00B2-\u00B5\u00B7-\u00BA]/,/[\u00BC-\u00BE\u00C0-\u00D6\u00D8-\u00F6\u00F8-\u00FF]/,/[\u0100-\u02FF\u0370-\u167F\u1681-\u180D\u180F-\u1DBF]/,/[\u1E00-\u1FFF]/,/[\u200B-\u200D\u202A-\u202E\u203F-\u2040\u2054\u2060-\u206F]/,/[\u2070-\u20CF\u2100-\u218F\u2460-\u24FF\u2776-\u2793]/,/[\u2C00-\u2DFF\u2E80-\u2FFF]/,/[\u3004-\u3007\u3021-\u302F\u3031-\u303F\u3040-\uD7FF]/,/[\uF900-\uFD3D\uFD40-\uFDCF\uFDF0-\uFE1F\uFE30-\uFE44]/,/[\uFE47-\uFEFE\uFF00-\uFFFD]/),f=n(h,/\d/,/[\u0300-\u036F\u1DC0-\u1DFF\u20D0-\u20FF\uFE20-\uFE2F]/),w=t(h,f,"*"),y=t(/[A-Z]/,f,"*"),g=["autoclosure",t(/convention\(/,n("swift","block","c"),/\)/),"discardableResult","dynamicCallable","dynamicMemberLookup","escaping","frozen","GKInspectable","IBAction","IBDesignable","IBInspectable","IBOutlet","IBSegueAction","inlinable","main","nonobjc","NSApplicationMain","NSCopying","NSManaged",t(/objc\(/,w,/\)/),"objc","objcMembers","propertyWrapper","requires_stored_property_inits","resultBuilder","testable","UIApplicationMain","unknown","usableFromInline"],E=["iOS","iOSApplicationExtension","macOS","macOSApplicationExtension","macCatalyst","macCatalystApplicationExtension","watchOS","watchOSApplicationExtension","tvOS","tvOSApplicationExtension","swift"]
    ;return e=>{const F={match:/\s+/,relevance:0},h=e.COMMENT("/\\*","\\*/",{
    contains:["self"]}),v=[e.C_LINE_COMMENT_MODE,h],A={match:[/\./,n(...s,...u)],
    className:{2:"keyword"}},N={match:t(/\./,n(...r)),relevance:0
    },C=r.filter((e=>"string"==typeof e)).concat(["_|0"]),D={variants:[{
    className:"keyword",
    match:n(...r.filter((e=>"string"!=typeof e)).concat(c).map(i),...u)}]},k={
    $pattern:n(/\b\w+/,/#\w+/),keyword:C.concat(m),literal:o},B=[A,N,D],_=[{
    match:t(/\./,n(...p)),relevance:0},{className:"built_in",
    match:t(/\b/,n(...p),/(?=\()/)}],S={match:/->/,relevance:0},M=[S,{
    className:"operator",relevance:0,variants:[{match:b},{match:`\\.(\\.|${d})+`}]
    }],x="([0-9a-fA-F]_*)+",I={className:"number",relevance:0,variants:[{
    match:"\\b(([0-9]_*)+)(\\.(([0-9]_*)+))?([eE][+-]?(([0-9]_*)+))?\\b"},{
    match:`\\b0x(${x})(\\.(${x}))?([pP][+-]?(([0-9]_*)+))?\\b`},{
    match:/\b0o([0-7]_*)+\b/},{match:/\b0b([01]_*)+\b/}]},L=(e="")=>({
    className:"subst",variants:[{match:t(/\\/,e,/[0\\tnr"']/)},{
    match:t(/\\/,e,/u\{[0-9a-fA-F]{1,8}\}/)}]}),O=(e="")=>({className:"subst",
    match:t(/\\/,e,/[\t ]*(?:[\r\n]|\r\n)/)}),T=(e="")=>({className:"subst",
    label:"interpol",begin:t(/\\/,e,/\(/),end:/\)/}),$=(e="")=>({begin:t(e,/"""/),
    end:t(/"""/,e),contains:[L(e),O(e),T(e)]}),j=(e="")=>({begin:t(e,/"/),
    end:t(/"/,e),contains:[L(e),T(e)]}),P={className:"string",
    variants:[$(),$("#"),$("##"),$("###"),j(),j("#"),j("##"),j("###")]},K={
    match:t(/`/,w,/`/)},z=[K,{className:"variable",match:/\$\d+/},{
    className:"variable",match:`\\$${f}+`}],q=[{match:/(@|#)available/,
    className:"keyword",starts:{contains:[{begin:/\(/,end:/\)/,keywords:E,
    contains:[...M,I,P]}]}},{className:"keyword",match:t(/@/,n(...g))},{
    className:"meta",match:t(/@/,w)}],U={match:a(/\b[A-Z]/),relevance:0,contains:[{
    className:"type",
    match:t(/(AV|CA|CF|CG|CI|CL|CM|CN|CT|MK|MP|MTK|MTL|NS|SCN|SK|UI|WK|XC)/,f,"+")
    },{className:"type",match:y,relevance:0},{match:/[?!]+/,relevance:0},{
    match:/\.\.\./,relevance:0},{match:t(/\s+&\s+/,a(y)),relevance:0}]},Z={
    begin:/</,end:/>/,keywords:k,contains:[...v,...B,...q,S,U]};U.contains.push(Z)
    ;const V={begin:/\(/,end:/\)/,relevance:0,keywords:k,contains:["self",{
    match:t(w,/\s*:/),keywords:"_|0",relevance:0
    },...v,...B,..._,...M,I,P,...z,...q,U]},W={begin:/</,end:/>/,contains:[...v,U]
    },G={begin:/\(/,end:/\)/,keywords:k,contains:[{
    begin:n(a(t(w,/\s*:/)),a(t(w,/\s+/,w,/\s*:/))),end:/:/,relevance:0,contains:[{
    className:"keyword",match:/\b_\b/},{className:"params",match:w}]
    },...v,...B,...M,I,P,...q,U,V],endsParent:!0,illegal:/["']/},R={
    match:[/func/,/\s+/,n(K.match,w,b)],className:{1:"keyword",3:"title.function"},
    contains:[W,G,F],illegal:[/\[/,/%/]},X={
    match:[/\b(?:subscript|init[?!]?)/,/\s*(?=[<(])/],className:{1:"keyword"},
    contains:[W,G,F],illegal:/\[|%/},H={match:[/operator/,/\s+/,b],className:{
    1:"keyword",3:"title"}},J={begin:[/precedencegroup/,/\s+/,y],className:{
    1:"keyword",3:"title"},contains:[U],keywords:[...l,...o],end:/}/}
    ;for(const e of P.variants){const a=e.contains.find((e=>"interpol"===e.label))
    ;a.keywords=k;const t=[...B,..._,...M,I,P,...z];a.contains=[...t,{begin:/\(/,
    end:/\)/,contains:["self",...t]}]}return{name:"Swift",keywords:k,
    contains:[...v,R,X,{beginKeywords:"struct protocol class extension enum actor",
    end:"\\{",excludeEnd:!0,keywords:k,contains:[e.inherit(e.TITLE_MODE,{
    className:"title.class",begin:/[A-Za-z$_][\u00C0-\u02B80-9A-Za-z$_]*/}),...B]
    },H,J,{beginKeywords:"import",end:/$/,contains:[...v],relevance:0
    },...B,..._,...M,I,P,...z,...q,U,V]}}})());hljs.registerLanguage("python",(()=>{"use strict";function e(e){
    return e?"string"==typeof e?e:e.source:null}function n(...n){
    return n.map((n=>e(n))).join("")}const t="[a-zA-Z]\\w*",a=(t,a,s={})=>{
    const i=((e,...n)=>{const t=Object.create(null);for(const n in e)t[n]=e[n]
    ;return n.forEach((e=>{for(const n in e)t[n]=e[n]})),t})({scope:"comment",
    begin:t,end:a,contains:[]},s);i.contains.push({scope:"doctag",
    begin:"[ ]*(?=(TODO|FIXME|NOTE|BUG|OPTIMIZE|HACK|XXX):)",
    end:/(TODO|FIXME|NOTE|BUG|OPTIMIZE|HACK|XXX):/,excludeBegin:!0,relevance:0})
    ;const r=function(...n){return"("+((e=>{const n=e[e.length-1]
    ;return"object"==typeof n&&n.constructor===Object?(e.splice(e.length-1,1),n):{}
    })(n).capture?"":"?:")+n.map((n=>e(n))).join("|")+")"
    }("I","a","is","so","us","to","at","if","in","it","on",/[A-Za-z]+['](d|ve|re|ll|t|s|n)/,/[A-Za-z]+[-][a-z]+/,/[A-Za-z][a-z]{2,}/)
    ;return i.contains.push({begin:n(/[ ]+/,"(",r,/[.]?[:]?([.][ ]|[ ])/,"){3}")}),i
    };return a("//","$"),a("/\\*","\\*/"),a("#","$"),e=>{const a={
    $pattern:/[A-Za-z]\w+|__\w+__/,
    keyword:["and","as","assert","async","await","break","class","continue","def","del","elif","else","except","finally","for","from","global","if","import","in","is","lambda","nonlocal|10","not","or","pass","raise","return","try","while","with","yield"],
    built_in:["__import__","abs","all","any","ascii","bin","bool","breakpoint","bytearray","bytes","callable","chr","classmethod","compile","complex","delattr","dict","dir","divmod","enumerate","eval","exec","filter","float","format","frozenset","getattr","globals","hasattr","hash","help","hex","id","input","int","isinstance","issubclass","iter","len","list","locals","map","max","memoryview","min","next","object","oct","open","ord","pow","print","property","range","repr","reversed","round","set","setattr","slice","sorted","staticmethod","str","sum","super","tuple","type","vars","zip"],
    literal:["__debug__","Ellipsis","False","None","NotImplemented","True"],
    type:["Any","Callable","Coroutine","Dict","List","Literal","Generic","Optional","Sequence","Set","Tuple","Type","Union"]
    },s={className:"meta",begin:/^(>>>|\.\.\.) /},i={className:"subst",begin:/\{/,
    end:/\}/,keywords:a,illegal:/#/},r={begin:/\{\{/,relevance:0},o={
    className:"string",contains:[e.BACKSLASH_ESCAPE],variants:[{
    begin:/([uU]|[bB]|[rR]|[bB][rR]|[rR][bB])?'''/,end:/'''/,
    contains:[e.BACKSLASH_ESCAPE,s],relevance:10},{
    begin:/([uU]|[bB]|[rR]|[bB][rR]|[rR][bB])?"""/,end:/"""/,
    contains:[e.BACKSLASH_ESCAPE,s],relevance:10},{
    begin:/([fF][rR]|[rR][fF]|[fF])'''/,end:/'''/,
    contains:[e.BACKSLASH_ESCAPE,s,r,i]},{begin:/([fF][rR]|[rR][fF]|[fF])"""/,
    end:/"""/,contains:[e.BACKSLASH_ESCAPE,s,r,i]},{begin:/([uU]|[rR])'/,end:/'/,
    relevance:10},{begin:/([uU]|[rR])"/,end:/"/,relevance:10},{
    begin:/([bB]|[bB][rR]|[rR][bB])'/,end:/'/},{begin:/([bB]|[bB][rR]|[rR][bB])"/,
    end:/"/},{begin:/([fF][rR]|[rR][fF]|[fF])'/,end:/'/,
    contains:[e.BACKSLASH_ESCAPE,r,i]},{begin:/([fF][rR]|[rR][fF]|[fF])"/,end:/"/,
    contains:[e.BACKSLASH_ESCAPE,r,i]},e.APOS_STRING_MODE,e.QUOTE_STRING_MODE]
    },l="[0-9](_?[0-9])*",c=`(\\b(${l}))?\\.(${l})|\\b(${l})\\.`,b={
    className:"number",relevance:0,variants:[{
    begin:`(\\b(${l})|(${c}))[eE][+-]?(${l})[jJ]?\\b`},{begin:`(${c})[jJ]?`},{
    begin:"\\b([1-9](_?[0-9])*|0+(_?0)*)[lLjJ]?\\b"},{
    begin:"\\b0[bB](_?[01])+[lL]?\\b"},{begin:"\\b0[oO](_?[0-7])+[lL]?\\b"},{
    begin:"\\b0[xX](_?[0-9a-fA-F])+[lL]?\\b"},{begin:`\\b(${l})[jJ]\\b`}]},d={
    className:"comment",begin:(g=/# type:/,n("(?=",g,")")),end:/$/,keywords:a,
    contains:[{begin:/# type:/},{begin:/#/,end:/\b\B/,endsWithParent:!0}]};var g
    ;const p={className:"params",variants:[{className:"",begin:/\(\s*\)/,skip:!0},{
    begin:/\(/,end:/\)/,excludeBegin:!0,excludeEnd:!0,keywords:a,
    contains:["self",s,b,o,e.HASH_COMMENT_MODE]}]};return i.contains=[o,b,s],{
    name:"Python",aliases:["py","gyp","ipython"],keywords:a,
    illegal:/(<\/|->|\?)|=>/,contains:[s,b,{begin:/\bself\b/},{beginKeywords:"if",
    relevance:0},o,d,e.HASH_COMMENT_MODE,{match:[/def/,/\s+/,t],scope:{1:"keyword",
    3:"title.function"},contains:[p]},{variants:[{
    match:[/class/,/\s+/,t,/\s*/,/\(\s*/,t,/\s*\)/]},{match:[/class/,/\s+/,t]}],
    scope:{1:"keyword",3:"title.class",6:"title.class.inherited"}},{
    className:"meta",begin:/^[\t ]*@/,end:/(?=#)|$/,contains:[b,p,o]}]}}})());hljs.registerLanguage("puppet",(()=>{"use strict";return e=>{
    const s=e.COMMENT("#","$"),r="([A-Za-z_]|::)(\\w|::)*",a=e.inherit(e.TITLE_MODE,{
    begin:r}),n={className:"variable",begin:"\\$"+r},i={className:"string",
    contains:[e.BACKSLASH_ESCAPE,n],variants:[{begin:/'/,end:/'/},{begin:/"/,end:/"/
    }]};return{name:"Puppet",aliases:["pp"],contains:[s,n,i,{beginKeywords:"class",
    end:"\\{|;",illegal:/=/,contains:[a,s]},{beginKeywords:"define",end:/\{/,
    contains:[{className:"section",begin:e.IDENT_RE,endsParent:!0}]},{
    begin:e.IDENT_RE+"\\s+\\{",returnBegin:!0,end:/\S/,contains:[{
    className:"keyword",begin:e.IDENT_RE},{begin:/\{/,end:/\}/,keywords:{
    keyword:"and case default else elsif false if in import enherits node or true undef unless main settings $string ",
    literal:"alias audit before loglevel noop require subscribe tag owner ensure group mode name|0 changes context force incl lens load_path onlyif provider returns root show_diff type_check en_address ip_address realname command environment hour monute month monthday special target weekday creates cwd ogoutput refresh refreshonly tries try_sleep umask backup checksum content ctime force ignore links mtime purge recurse recurselimit replace selinux_ignore_defaults selrange selrole seltype seluser source souirce_permissions sourceselect validate_cmd validate_replacement allowdupe attribute_membership auth_membership forcelocal gid ia_load_module members system host_aliases ip allowed_trunk_vlans description device_url duplex encapsulation etherchannel native_vlan speed principals allow_root auth_class auth_type authenticate_user k_of_n mechanisms rule session_owner shared options device fstype enable hasrestart directory present absent link atboot blockdevice device dump pass remounts poller_tag use message withpath adminfile allow_virtual allowcdrom category configfiles flavor install_options instance package_settings platform responsefile status uninstall_options vendor unless_system_user unless_uid binary control flags hasstatus manifest pattern restart running start stop allowdupe auths expiry gid groups home iterations key_membership keys managehome membership password password_max_age password_min_age profile_membership profiles project purge_ssh_keys role_membership roles salt shell uid baseurl cost descr enabled enablegroups exclude failovermethod gpgcheck gpgkey http_caching include includepkgs keepalive metadata_expire metalink mirrorlist priority protect proxy proxy_password proxy_username repo_gpgcheck s3_enabled skip_if_unavailable sslcacert sslclientcert sslclientkey sslverify mounted",
    built_in:"architecture augeasversion blockdevices boardmanufacturer boardproductname boardserialnumber cfkey dhcp_servers domain ec2_ ec2_userdata facterversion filesystems ldom fqdn gid hardwareisa hardwaremodel hostname id|0 interfaces ipaddress ipaddress_ ipaddress6 ipaddress6_ iphostnumber is_virtual kernel kernelmajversion kernelrelease kernelversion kernelrelease kernelversion lsbdistcodename lsbdistdescription lsbdistid lsbdistrelease lsbmajdistrelease lsbminordistrelease lsbrelease macaddress macaddress_ macosx_buildversion macosx_productname macosx_productversion macosx_productverson_major macosx_productversion_minor manufacturer memoryfree memorysize netmask metmask_ network_ operatingsystem operatingsystemmajrelease operatingsystemrelease osfamily partitions path physicalprocessorcount processor processorcount productname ps puppetversion rubysitedir rubyversion selinux selinux_config_mode selinux_config_policy selinux_current_mode selinux_current_mode selinux_enforced selinux_policyversion serialnumber sp_ sshdsakey sshecdsakey sshrsakey swapencrypted swapfree swapsize timezone type uniqueid uptime uptime_days uptime_hours uptime_seconds uuid virtual vlans xendomains zfs_version zonenae zones zpool_version"
    },relevance:0,contains:[i,s,{begin:"[a-zA-Z_]+\\s*=>",returnBegin:!0,end:"=>",
    contains:[{className:"attr",begin:e.IDENT_RE}]},{className:"number",
    begin:"(\\b0[0-7_]+)|(\\b0x[0-9a-fA-F_]+)|(\\b[1-9][0-9_]*(\\.[0-9_]+)?)|[0_]\\b",
    relevance:0},n]}],relevance:0}]}}})());hljs.registerLanguage("sql",(()=>{"use strict";function e(e){
    return e?"string"==typeof e?e:e.source:null}function r(...r){
    return r.map((r=>e(r))).join("")}function t(...r){return"("+((e=>{
    const r=e[e.length-1]
    ;return"object"==typeof r&&r.constructor===Object?(e.splice(e.length-1,1),r):{}
    })(r).capture?"":"?:")+r.map((r=>e(r))).join("|")+")"}return e=>{
    const n=e.COMMENT("--","$"),a=["true","false","unknown"],i=["bigint","binary","blob","boolean","char","character","clob","date","dec","decfloat","decimal","float","int","integer","interval","nchar","nclob","national","numeric","real","row","smallint","time","timestamp","varchar","varying","varbinary"],s=["abs","acos","array_agg","asin","atan","avg","cast","ceil","ceiling","coalesce","corr","cos","cosh","count","covar_pop","covar_samp","cume_dist","dense_rank","deref","element","exp","extract","first_value","floor","json_array","json_arrayagg","json_exists","json_object","json_objectagg","json_query","json_table","json_table_primitive","json_value","lag","last_value","lead","listagg","ln","log","log10","lower","max","min","mod","nth_value","ntile","nullif","percent_rank","percentile_cont","percentile_disc","position","position_regex","power","rank","regr_avgx","regr_avgy","regr_count","regr_intercept","regr_r2","regr_slope","regr_sxx","regr_sxy","regr_syy","row_number","sin","sinh","sqrt","stddev_pop","stddev_samp","substring","substring_regex","sum","tan","tanh","translate","translate_regex","treat","trim","trim_array","unnest","upper","value_of","var_pop","var_samp","width_bucket"],o=["create table","insert into","primary key","foreign key","not null","alter table","add constraint","grouping sets","on overflow","character set","respect nulls","ignore nulls","nulls first","nulls last","depth first","breadth first"],c=s,l=["abs","acos","all","allocate","alter","and","any","are","array","array_agg","array_max_cardinality","as","asensitive","asin","asymmetric","at","atan","atomic","authorization","avg","begin","begin_frame","begin_partition","between","bigint","binary","blob","boolean","both","by","call","called","cardinality","cascaded","case","cast","ceil","ceiling","char","char_length","character","character_length","check","classifier","clob","close","coalesce","collate","collect","column","commit","condition","connect","constraint","contains","convert","copy","corr","corresponding","cos","cosh","count","covar_pop","covar_samp","create","cross","cube","cume_dist","current","current_catalog","current_date","current_default_transform_group","current_path","current_role","current_row","current_schema","current_time","current_timestamp","current_path","current_role","current_transform_group_for_type","current_user","cursor","cycle","date","day","deallocate","dec","decimal","decfloat","declare","default","define","delete","dense_rank","deref","describe","deterministic","disconnect","distinct","double","drop","dynamic","each","element","else","empty","end","end_frame","end_partition","end-exec","equals","escape","every","except","exec","execute","exists","exp","external","extract","false","fetch","filter","first_value","float","floor","for","foreign","frame_row","free","from","full","function","fusion","get","global","grant","group","grouping","groups","having","hold","hour","identity","in","indicator","initial","inner","inout","insensitive","insert","int","integer","intersect","intersection","interval","into","is","join","json_array","json_arrayagg","json_exists","json_object","json_objectagg","json_query","json_table","json_table_primitive","json_value","lag","language","large","last_value","lateral","lead","leading","left","like","like_regex","listagg","ln","local","localtime","localtimestamp","log","log10","lower","match","match_number","match_recognize","matches","max","member","merge","method","min","minute","mod","modifies","module","month","multiset","national","natural","nchar","nclob","new","no","none","normalize","not","nth_value","ntile","null","nullif","numeric","octet_length","occurrences_regex","of","offset","old","omit","on","one","only","open","or","order","out","outer","over","overlaps","overlay","parameter","partition","pattern","per","percent","percent_rank","percentile_cont","percentile_disc","period","portion","position","position_regex","power","precedes","precision","prepare","primary","procedure","ptf","range","rank","reads","real","recursive","ref","references","referencing","regr_avgx","regr_avgy","regr_count","regr_intercept","regr_r2","regr_slope","regr_sxx","regr_sxy","regr_syy","release","result","return","returns","revoke","right","rollback","rollup","row","row_number","rows","running","savepoint","scope","scroll","search","second","seek","select","sensitive","session_user","set","show","similar","sin","sinh","skip","smallint","some","specific","specifictype","sql","sqlexception","sqlstate","sqlwarning","sqrt","start","static","stddev_pop","stddev_samp","submultiset","subset","substring","substring_regex","succeeds","sum","symmetric","system","system_time","system_user","table","tablesample","tan","tanh","then","time","timestamp","timezone_hour","timezone_minute","to","trailing","translate","translate_regex","translation","treat","trigger","trim","trim_array","true","truncate","uescape","union","unique","unknown","unnest","update","upper","user","using","value","values","value_of","var_pop","var_samp","varbinary","varchar","varying","versioning","when","whenever","where","width_bucket","window","with","within","without","year","add","asc","collation","desc","final","first","last","view"].filter((e=>!s.includes(e))),u={
    begin:r(/\b/,t(...c),/\s*\(/),relevance:0,keywords:{built_in:c}};return{
    name:"SQL",case_insensitive:!0,illegal:/[{}]|<\//,keywords:{
    $pattern:/\b[\w\.]+/,keyword:((e,{exceptions:r,when:t}={})=>{const n=t
    ;return r=r||[],e.map((e=>e.match(/\|\d+$/)||r.includes(e)?e:n(e)?e+"|0":e))
    })(l,{when:e=>e.length<3}),literal:a,type:i,
    built_in:["current_catalog","current_date","current_default_transform_group","current_path","current_role","current_schema","current_transform_group_for_type","current_user","session_user","system_time","system_user","current_time","localtime","current_timestamp","localtimestamp"]
    },contains:[{begin:t(...o),relevance:0,keywords:{$pattern:/[\w\.]+/,
    keyword:l.concat(o),literal:a,type:i}},{className:"type",
    begin:t("double precision","large object","with timezone","without timezone")
    },u,{className:"variable",begin:/@[a-z0-9]+/},{className:"string",variants:[{
    begin:/'/,end:/'/,contains:[{begin:/''/}]}]},{begin:/"/,end:/"/,contains:[{
    begin:/""/}]},e.C_NUMBER_MODE,e.C_BLOCK_COMMENT_MODE,n,{className:"operator",
    begin:/[-+*/=%^~]|&&?|\|\|?|!=?|<(?:=>?|<|>)?|>[>=]?/,relevance:0}]}}})());