// Presentation controls. Route and export logic remains in the main application.
const routePanel=document.getElementById('routePanel');
const settingsPanel=document.getElementById('settingsPanel');
function showBuilderPanel(settings){
 routePanel.hidden=settings;settingsPanel.hidden=!settings;
 for(const [id,active] of [['routeTab',!settings],['settingsTab',settings]]){
  const button=document.getElementById(id);button.classList.toggle('active',active);button.setAttribute('aria-pressed',String(active));
 }
 requestAnimationFrame(()=>map.invalidateSize());
}
document.getElementById('routeTab').addEventListener('click',()=>showBuilderPanel(false));
document.getElementById('settingsTab').addEventListener('click',()=>showBuilderPanel(true));
function showMetrics(visible){
 document.getElementById('routeMetrics').hidden=!visible;
 document.getElementById('metricsToggle').setAttribute('aria-expanded',String(visible));
}
document.getElementById('metricsToggle').addEventListener('click',()=>{
 if(!document.getElementById('statsSection').style.display){showMetrics(document.getElementById('routeMetrics').hidden);}
 else{toast('Confirm your route, then select Preview route to see its metrics.');}
});
document.getElementById('closeMetrics').addEventListener('click',()=>showMetrics(false));
document.getElementById('dismissProgress').addEventListener('click',()=>{if(!state.processing)document.getElementById('exportProgressPanel').style.display='none';});
const metricsObserver=new MutationObserver(()=>{
 if(document.getElementById('statsSection').style.display!=='none')showMetrics(true);
});
metricsObserver.observe(document.getElementById('statsSection'),{attributes:true,attributeFilter:['style']});
window.addEventListener('resize',()=>map.invalidateSize());
document.addEventListener('keydown',event=>{if(event.key==='Escape'){showMetrics(false);if(!state.processing)document.getElementById('exportProgressPanel').style.display='none';}});
