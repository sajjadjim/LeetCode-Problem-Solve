import { Suspense } from 'react';
import './App.css'
import Bottles from './components/bottles';

function App() {

  // Data Load Here 
const bottleData = async() =>{
  const res = await fetch('https://raw.githubusercontent.com/sajjadjim/JSON-Data-Folder/refs/heads/main/bottle-data-all')
  return res.json()
}
const bottlePromise = bottleData()

  return (
    <>
<Suspense fallback={<h1>Data Loading....</h1>}>
<Bottles bottlePromise={bottlePromise}></Bottles>
</Suspense>
    </>
  )
}

export default App
