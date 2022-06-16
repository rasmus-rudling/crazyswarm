import "./App.css";

import DataCollection from "pages/DataCollection";
import ResetEvaluation from "pages/ResetEvaluation";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";

const App = () => {
  return (
    <Router>
      <Routes>
        <Route path="/reset" element={<ResetEvaluation />} />
        <Route path="/" element={<DataCollection />} />
      </Routes>
    </Router>
  );
};

export default App;
